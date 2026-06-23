# F28069 Pack BMS — SOC 헌팅 펌웨어 분석 및 해결방안 검토

**대상:** `D:\202 15S2P_PackBMSR02\F28069PackBMS` (TI F28069 / C2000, EVE LF230Ah, 15S2P, Pack 460Ah)
**활성 빌드:** `EVE24060Ah = 1` → SOC 엔진 `CalEVE240AhSocHandle()` (`SysSoure/BATAlgorithm.c`)
**결론:** **개선 가능 — 난이도 낮음, 리스크 낮음.** 핵심 수정은 `BATAlgorithm.c` 1개 함수에 국한되며, 해결 로직(평탄구간 가드)이 이미 부팅 초기화 코드에 구현돼 있어 그대로 런타임에 이식하면 됩니다.

---

## 1. 한 줄 결론

SOC 헌팅(±64~68% 점프)은 **셀 화학이나 센서 문제가 아니라, 휴지(저전류) 시 SOC를 OCV 테이블 값으로 무조건 전면 덮어쓰는 런타임 로직의 결함**입니다. LFP의 평탄한 OCV-SOC 곡선 위에서 이 동작이 수 mV 오차를 수십 %p SOC 오차로 증폭시킵니다. 부팅 코드에는 이를 막는 가드가 이미 있으나 런타임 경로에는 빠져 있어, **동일 가드를 런타임에 적용**하면 해결됩니다.

---

## 2. 펌웨어 SOC 로직 구조

`main.c`(약 1044~1051행)가 매 사이클 BMS 동작 중 다음을 `CalEVE240AhSocHandle()`에 입력합니다.

```c
EV240AhSocRegs.CellAgvVoltageF = SysRegs.SysCellAgvVoltageF; // = PackVoltage/CellSize (실시간 단자전압!)
EV240AhSocRegs.SysSoCCTF       = SysRegs.SysPackCurrentF;    // 전류(부호: 충전+/방전-)
EV240AhSocRegs.SysSoCCTAbsF    = SysRegs.SysPackCurrentAsbF; // |전류|
CalEVE240AhSocHandle(&EV240AhSocRegs);
```

엔진은 50ms 주기로 `|전류|` 기준 2가지 모드를 전환합니다 (`BATAlgorithm.c` 945~1047행).

| 모드 | 조건 | 동작 |
|---|---|---|
| **CalMeth=1** (전류적산) | `|I| ≥ 2.5A` | `SysPackSOCF = SysSocInitF + ∫I·dt/368Ah×100`. 정상·정확. |
| **CalMeth=0** (OCV 재초기화) | `|I| < 2.5A` 가 **5분(6000×50ms)** 지속 | `CalEVE240AhSocInit()`로 OCV 테이블 룩업 → **`SysPackSOCF = SysSocInitF`로 전면 덮어쓰기**. |

OCV 테이블(`EVE_LF230_OCV_TABLE`)의 평탄구간 압축이 문제의 본질입니다.

```
3.295V → 25.00%      ┐
3.318V → 50.00%      │  Disp SOC 25→93.75% (약 69%p)가
3.333V → 80.00%      │  단 45mV(3.295~3.340V) 안에 압축
3.340V → 93.75%      ┘  → 1mV ≈ 1.5%p SOC
3.360V → 100%  (급경사: 신뢰 가능 구간)
3.160V → 0%    (급경사: 신뢰 가능 구간)
```

---

## 3. 근본 원인 — 코드 ↔ BLF 관측 현상 매핑

### 원인 ① 런타임 OCV 재초기화에 "평탄구간 가드"가 없음 (핵심)

`BATAlgorithm.h`(247~250행)에 **설계 의도**가 명시돼 있습니다:

> *Flat zone (Disp 25~93.75%) → use NVR / Sharp zone (0~25%, 93.75~100%) → use OCV*

그리고 부팅 초기화(`main.c` 397~417행)에는 이 의도가 **정확히 구현**돼 있습니다:

```c
NVRAM_AZoneReadHandler(&NVRZoneARDRegs);
EV240AhSocRegs.SysSocInitF = (float32)(NVRZoneARDRegs.LastSOC/10.0f);
if((SysSocInitF >= NVR_SOC_VALID_LO /*22%*/) && (SysSocInitF < NVR_SOC_VALID_HI /*92%*/))
{   SysSocInitRule = SOC_ZONE_NVR;  }      // 평탄구간 → NVR 신뢰 (OCV 금지)
else
{   CalEVE240AhSocInit(...);  SysSocInitRule = SOC_ZONE_cellVolt; } // 급경사 → OCV
```

**그런데 런타임 경로(`CalEVE240AhSocHandle`, 1001~1010행)에는 이 가드가 빠져 있습니다:**

```c
if(P->SoCStateRegs.bit.CalMeth == 0u)   /* 휴지 */
{
    CalEVE240AhSocInit(P);              // ← 구간 무관하게 무조건 OCV 룩업
    P->SysPackSOCF = P->SysSocInitF;    // ← 적산값을 통째로 덮어씀 (점프 발생)
}
```

→ 평탄구간(3.295~3.340V)에서도 휴지 5분이면 SOC를 OCV 값으로 강제 교체. **부팅 로직과 런타임 로직의 불일치**가 결함의 핵심입니다.

### 원인 ② OCV에 완화(relaxation)·IR 보정이 없음 — "결정적 증거"

OCV 룩업에 쓰는 `CellAgvVoltageF`는 **실시간 단자전압/셀수**입니다. 방금까지 방전하던 셀의 분극이 풀리지 않은 전압을 OCV로 간주합니다.

> **결정적 일치:** 방전1에서 보고 SOC가 정확히 **19.0%**로 붕괴했는데, OCV 테이블에서 **3.280V → 18.75%**. 즉 휴지 판정 구간(전류 −1A)에서 분극으로 셀 전압이 ~3.28V로 가라앉아 있었고, 가드 없는 OCV 룩업이 이를 19%로 매핑해 87% 적산값을 덮어쓴 것입니다. 이후 전압이 3.33~3.34V로 회복된 시점의 재룩업이 +68.5% 복귀(→89%)를 만들었습니다.

### 원인 ③ 전면 덮어쓰기 (슬루레이트 제한 없음)

`SysPackSOCF = SysSocInitF`는 한 스텝에 수십 %p를 점프시킵니다. 점진 수렴·필터가 없어 관측된 계단형 ±64~68% 점프로 그대로 표출됩니다.

### 원인 ④ 부팅 시드 신뢰성

충전 로그에서 SOC가 내내 25~31%에 정체한 것은, 부팅 시드(`SysSocInitF`)가 ~25%로 잘못 잡힌 뒤 적산만 +4.5% 누적했기 때문입니다(쿨롱카운팅 자체는 정상, 순변화 −3.9% vs 실측 −4.2% 일치). NVR이 무효이거나 범위를 벗어나면 평탄 plateau OCV로 재시드되는데, 이 재시드값 자체가 평탄구간이라 신뢰 불가입니다.

### 관측 현상 요약

| 로그 | 관측 | 펌웨어 원인 |
|---|---|---|
| CHARGE | SOC 25~31% 정체 → 종료 휴지 시 **+63.7% 점프**(94.5%) | 잘못된 부팅 시드(원인④) + 휴지 OCV 재초기화(원인①) |
| DISCHARGE1 | 87% → **19% 붕괴**(3.28V) → **89% 복귀** | 가드 없는 OCV(①) + 비완화 전압(②) + 덮어쓰기(③) |
| DISCHARGE2 | 88.9→87.8% 정상 | 휴지 5분 미도달 → CalMeth=1 적산 유지(정상 경로) |

---

## 3.5 주석이 말해주는 개발 이력 — "미완성 리팩터링" (핵심 보강)

코드 주석을 추적하면, 본 증상은 우발적 버그가 아니라 **절반만 적용된 리팩터링**의 결과임이 드러납니다.

### 증거 ① 헌팅의 직접적 출처 — `BATAlgorithm.c:978-979`

```c
P->SoCStateRegs.bit.CalMeth = 0u;   /* 휴지 5분 이상 -> OCV 재초기화 */  ← 현재 활성
//P->SoCStateRegs.bit.CalMeth = 1u; /* (구) 적산 유지 - 비활성 */         ← 과거 동작
```

**구(舊) 동작은 휴지 5분 이후에도 `CalMeth=1`(전류적산 유지)** 였습니다. 이를 주석 처리하고 `CalMeth=0`(OCV 재초기화)으로 교체한 변경이 곧 점프의 시작점입니다. 평탄구간 가드 없이 이 전환만 도입되어, 휴지 시마다 평탄 plateau OCV로 SOC를 덮어쓰게 됐습니다.

### 증거 ② 가드는 부팅에만, 런타임은 방치 — `main.c:370-417`

구 런타임 zone 핸들러(`SysCalSocZoneHandle()` — NVR vs cellVolt 구간 판정)는 **"폐지"되어 주석 처리**(main.c:373-394)됐고, 그 올바른 NVR-신뢰 로직은 **부팅 경로에만** 새로 구현됐습니다(main.c:396-417). 그 결과:

- **부팅 경로:** NVR 22~92%(평탄) → NVR 신뢰 / 범위 밖(급경사) → OCV. ✅ 올바름
- **런타임 휴지 경로(`BATAlgorithm.c`):** 구간 무관 OCV 덮어쓰기. ❌ 가드 누락

### 증거 ③ 활성 코드가 개발자 정책 주석과 모순 — `main.c:1052-1058`

```
* 비고 : ... BMS ON 유지 중에는 전류적산 결과(SysPackSOCF)를 그대로 사용한다.
*        - SOC 재시드는 BMS OFF->ON 부팅 init에서만 수행
```

→ 명시된 정책은 **"런타임 재시드 금지, 적산값 사용"**. 그러나 활성 코드(978행)는 런타임 휴지 시 OCV로 재시드합니다. **정책 ↔ 구현 불일치**가 핵심이며, 수정 방향이 명확함을 의미합니다: *런타임을 정책(또는 부팅 로직)에 맞추면 됨.*

> 함의: 가장 안전·정확한 수정은 새 알고리즘 도입이 아니라 **이미 코드에 존재하는 의도(구 동작/부팅 가드/정책 주석)를 런타임에 일관 적용**하는 것입니다.

---

## 4. 해결 방안 (구현 레벨)

수정 범위는 사실상 `BATAlgorithm.c`의 `CalEVE240AhSocHandle()` 1개 함수입니다.

### 방안 A0 — 1줄 수정 (가장 보수적, 정책 주석과 일치)

`main.c:1054` 정책("런타임 중 적산값 사용, 재시드는 부팅에서만")을 그대로 따른다면, `BATAlgorithm.c:978`에서 휴지 시 OCV 재초기화 자체를 비활성화(구 동작 복원)합니다.

```c
// BATAlgorithm.c : 978
//P->SoCStateRegs.bit.CalMeth = 0u;   /* 비활성: 런타임 OCV 재초기화 금지 */
  P->SoCStateRegs.bit.CalMeth = 1u;   /* 휴지에도 적산 유지 (구 동작 = 정책 부합) */
```

장점: 변경 최소·즉효. 단점: 만충/만방 급경사 구간의 자동 재앵커링까지 사라지므로, 장기 적산 드리프트 보정은 부팅 NVR 재시드에만 의존. → **방안 A 권장(급경사 OCV는 유지).**

### 방안 A — 평탄구간 가드 + 슬루레이트 제한 (권장, 즉시 적용 가능)

부팅에 이미 있는 가드를 런타임 CalMeth==0 분기에 이식하고, 덮어쓰기를 점진 수렴으로 바꿉니다.

```c
/* BATAlgorithm.h 에 추가 */
#define SOC_SLEW_MAX_PCT   0.05f   /* 50ms당 최대 SOC 보정량(%) → 약 1%/초 */
#ifndef CLAMP
#define CLAMP(x,lo,hi)  ((x)<(lo)?(lo):((x)>(hi)?(hi):(x)))
#endif

/* CalEVE240AhSocHandle() 의 CalMeth==0 분기 교체 */
if(P->SoCStateRegs.bit.CalMeth == 0u)        /* 휴지: OCV 재초기화 후보 */
{
    float32 vcell = P->CellAgvVoltageF;

    /* (1) 평탄구간 가드: 급경사 구간에서만 OCV 신뢰 (부팅 로직과 동일 기준) */
    if((vcell <= V_FlatStartF) || (vcell >= V_FlatEndF))   /* ≤3.295V or ≥3.340V */
    {
        CalEVE240AhSocInit(P);               /* OCV 테이블 룩업 → SysSocInitF */

        /* (2) 슬루레이트 제한: 한 번에 덮지 않고 점진 수렴 → 계단 점프 제거 */
        float32 err  = P->SysSocInitF - P->SysPackSOCF;
        P->SysPackSOCF += CLAMP(err, -SOC_SLEW_MAX_PCT, +SOC_SLEW_MAX_PCT);
        P->SysSocInitF  = P->SysPackSOCF;    /* 적산 재개 기준점 동기화 */
    }
    else
    {
        /* (3) 평탄구간: OCV 무시, 직전 SOC(=적산/NVR) 유지. 기준점만 동기화 */
        P->SysSocInitF = P->SysPackSOCF;
    }
    P->state = SOC_STATE_SOSINIT;
}
```

효과: 방전1의 19% 붕괴(원인①②③) 직접 차단, 충전 종료 +63.7% 점프도 평탄구간이므로 억제. **방안 A만으로 관측된 모든 대형 점프가 제거**됩니다.

### 방안 B — 휴지 OCV에 완화 대기 + IR 보정 추가 (정확도 강화)

급경사 구간에서 OCV를 쓸 때 신뢰도를 더 높입니다.

- **완화 대기:** `|I|<2.5A` 5분 조건에 더해 `|dV/dt|`가 작아질 때까지(예: <0.2mV/s) 대기 후에만 OCV 채택. 분극이 풀린 진짜 OCV를 사용.
- **IR 보정:** 부하 잔류 시 `OCV ≈ V_term − I·Rint` (BLF 회귀 추정 Rint≈4.5~5.3mΩ/팩, 셀 환산 ~0.6mΩ)로 보정 후 룩업. `BPA_CellRegs`(셀 저항) 신호를 이미 측정하므로 활용 가능.

### 방안 C — 부팅 시드 강건화

NVR이 무효/범위 밖이고 현재 평탄구간이면, plateau OCV로 시드하지 말고 (a) 직전 NVR 유지 또는 (b) 완화 대기 후 급경사 진입 시 보정. `main.c` 부팅 분기에 평탄구간 예외만 추가하면 됩니다(원인④ 해소).

### 방안 D — (선택) 적산+OCV 융합 추정기

장기적으로 1차 RC 등가모델 + 보정 게인(간이 칼만) 도입. 평탄구간은 적산 신뢰, 급경사·완화 OCV로만 소폭 보정. F28069 부동소수 연산으로 충분히 구현 가능하나, 위 A~C로 현 증상은 해소되므로 우선순위는 후순위.

### 권장 적용 순서

1. **방안 A** (필수, 즉효 — 급경사 OCV 유지 + 평탄구간 가드 + 슬루) → 2. **방안 C** (부팅 안정화) → 3. **방안 B** (정확도) → 4. 방안 D(여력 시)
   - 최소 변경만 원하면 **방안 A0**(1줄)로 즉시 점프 제거 후, 단계적으로 A→B 적용 가능.

---

## 5. 검증 방법 & 리스크

**검증**
- 본 BLF 3종을 입력으로 수정 로직을 PC에서 재현(오프라인 시뮬레이션)해 점프 소멸 확인 — 기존 디코딩 데이터로 즉시 가능.
- 실차 재시험: 동일 충·방전 프로파일 + 중간 휴지 5분 이상 구간을 의도적으로 포함시켜 점프 부재 확인.
- 급경사 구간(만충/만방 부근) 진입 시 OCV 보정이 정상 동작하는지 별도 확인.

**리스크 (낮음)**
- 평탄구간 OCV를 억제하므로, 적산 드리프트가 장기 누적될 수 있음 → 만충/만방(급경사) 도달 시 자동 리앵커링으로 상쇄됨(전류 0.002% 정밀, SOC ±5% 사양 내, `main.c` 1057행 주석). 주기적 완전충전 또는 NVR 저장 시퀀스로 보강 권장.
- 슬루레이트 제한 도입 시 정당한 큰 보정이 수초 지연될 수 있으나, 표시 SOC 안정성 이득이 훨씬 큼.

---

## 6. 부록 — 핵심 파라미터

| 항목 | 값 | 위치 |
|---|---|---|
| 활성 셀 빌드 | `EVE24060Ah=1` | BATAlgorithm.h:35 |
| 적산/OCV 전환 전류 | `C_SocInitCTVaule = 2.5A` | BATAlgorithm.h:37 |
| 휴지→OCV 재초기화 대기 | 6000 × 50ms = **5분** | BATAlgorithm.c:975 |
| 정규화 용량 | 368Ah (460Ah × DoD80%) | BATAlgorithm.c:749 |
| 평탄구간 경계 | `V_FlatStartF=3.295V`, `V_FlatEndF=3.340V` | BATAlgorithm.h:262-263 |
| NVR 신뢰범위 | 22% ~ 92% | main.c:19-20 |
