# 충전 제어 로직 명세 (case 10, main.c)

## 1. 개요

BMS가 충전기로 송신하는 **CV 종단 전압**(`CharCONSTVolt`, 단위 0.1V)을 SOC 기반 선형 매핑으로 동적 제어한다. CC 구간 연장과 CV 매끄러운 전환을 통해 충전 시간 단축과 만충 정확도를 확보한다.

## 2. 송신 패킷

| CAN ID | 주기 | 송신 변수 | 단위 |
|:---:|:---:|---|:---:|
| 0x61E | 300ms | `CharCONSTVolt`, `CahrConstantCurrt`, `SysPackPT`, `SysPackCT` | 전압 0.1V, 전류 0.1A |
| 0x61F | 300ms | `ChargerStateRegs` (BSACHAEnable, BatNRly, BatPRly 등) | bit |

## 3. 정상 충전 게이트

```c
if((CHAComStatus == 1u) && (VCUComStatus == 0u))
```

- **충전기 연결**(CHA=1) AND **VCU 미연결**(VCU=0)일 때만 SOC 기반 동적 제어 작동
- 둘 다 1이면 시스템 오류 (현재 별도 가드 없음, 미해결 과제)
- 게이트 미충족 시 `CharCONSTVolt = 520`

## 4. SOC 기반 분기

| 분기 | 조건 | 출력 | 의도 |
|:---:|---|:---:|---|
| 1 | `SOC < 85%` | 520 (52.0V) | CC 유지 (저~중 SOC에서 빠른 충전) |
| 2 | `85% ≤ SOC < 93%` (명시적 AND) | `520 - (SOC-85)×2.0` | CC→CV 매끄러운 선형 강하 |
| 3 | `SOC ≥ 93%` (별도 if) | 504 (50.4V) | CV 안착 |

### 분기 2 선형 식

```c
CharCONSTVolt = (Uint16)(520.0F - (SysSOCF - 85.0F) * 2.0F);
```

- **기울기**: (520 − 504) ÷ (93 − 85) = **2.0 per %**
- SOC 85% → 520, 89% → 512, 92.9% → 504, 점프 없음

## 5. 안전 핫픽스 (식 폭주 방지)

```c
if(CharCONSTVolt > 521u) { CharCONSTVolt = 504u; }
```

- 분기 후 일괄 적용
- **평상시 미발동** (옵션 A 식의 최대 결과 520)
- 미래에 식이 잘못 바뀌어 521 초과 시 충전기 거부 방지용 비상망

## 6. 충전 종료 조건 (PWRRlyHoldHandle, DSP28x_Project.c)

```c
if(SysSOCF >= CHARGE_END_SOC)        /* 100.0F */
{
    if(SysPackCurrentAsbF <= CHARGE_END_CURRENT)   /* 9.6F = 0.04C of 240Ah */
    {
        BSACHAEnable = 0u;     // 충전 허가 해제
    }
}
```

## 7. BSACHAEnable 동작 명세

| 조건 | 값 | 위치 |
|---|:---:|---|
| BMS ON 즉시 초기화 | 1 | SysVarINIT (DSP28x_Project.c:130) |
| READY 진입 | 1 | main.c:405 |
| 정상 충전 중 | 1 유지 | (변경 없음) |
| 종료 조건 충족 (SOC≥100 & 전류≤9.6A) | 0 | PWRRlyHoldHandle |

시스템 오류(VCU=1 & CHA=1) 시 충전 차단은 BSACHAEnable이 아닌 **다른 방식(릴레이 OFF)으로 별도 처리 예정** (미구현).

## 8. 관련 매크로

| 매크로 | 값 | 정의 위치 |
|---|:---:|---|
| `CHARGE_END_SOC` | 100.0F | DSP28x_Project.c:35 |
| `CHARGE_END_CURRENT` | 9.6F (0.04C, 240Ah) | DSP28x_Project.c:36 |

## 9. 충전기 사양 (참고)

- CC 전류: 30A (코드 송신 `CahrConstantCurrt = 300` 또는 250 = 25A)
- 종단 전류: 2A (충전기 자체 종료)
- 최대 출력 전압: 54V (사용자 확인, 52V 송신은 수용 가능)

## 10. 동작 흐름 SOC별 시뮬

| SOC | 진입 분기 | CharCONSTVolt | 비고 |
|:---:|:---:|:---:|---|
| < 85 | 1 | 520 | CC, 52.0V |
| 85 | 2 (식 520) | 520 | CC |
| 87 | 2 (식 516) | 516 | 51.6V |
| 89 | 2 (식 512) | 512 | 51.2V |
| 91 | 2 (식 508) | 508 | 50.8V |
| 92.9 | 2 (식 504) | 504 | CV 진입점 |
| ≥ 93 | 3 | 504 | CV 안착, 50.4V |
| 게이트 미충족 (else) | — | 520 | 미충전/오류 기본값 |

## 11. 변경 이력 (2026-05)

- 50.4V 고정 종단 → 동적 CV (상대오프셋, 팩+1V CC) → SOC 절대값 분기로 전환
- BMS OFF→ON 시 BSACHAEnable=0 버그 → SysVarINIT에서 1로 초기화
- 셀 전압 보정/만충 SOC 리셋 시도했으나 사용자 요청으로 미적용 (LFP 평탄대 한계)
- 시스템 오류 가드(VCU=1&CHA=1 차단) 시도했으나 사용자 요청으로 미적용

## 12. 미해결 과제

- SOCINIT 오류 시 SOC 표시 정확화 (LFP 평탄대로 충전 중 보정 어려움)
- 시스템 오류 시 충전 차단 (BSACHAEnable이 아닌 릴레이 방식으로)
- 분기 2 점프 검증 (현장에서 521 헌팅 발생 시 빌드/업로드 확인)
