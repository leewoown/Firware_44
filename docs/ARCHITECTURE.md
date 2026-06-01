# F28069 Pack BMS 펌웨어 아키텍처

> 대상 MCU: TI TMS320F28069 (C2000, 90 MHz)
> 개발환경: Code Composer Studio 11.0.0 / C2000 codegen 20.2.5.LTS
> 용도: 현대자동차 수소지게차 / 연료전지 파워팩용 배터리 관리 시스템(BMS)
> 구성: **80V 메인 팩(24S, Farasis 52Ah NCM)** + **12V 보조 팩(4S, Frey 60Ah LFP)**

---

## 1. 개요

본 펌웨어는 두 개의 배터리 팩을 동시에 관리하는 단일 칩 BMS이다.

| 항목 | 80V 메인 팩 | 12V 보조 팩 |
|------|-------------|-------------|
| 셀 구성 | 24직렬 (24S1P) | 4직렬 (4S) |
| 셀 화학 | NCM (Farasis 52Ah) | LFP (Frey 60Ah) |
| 정격 전압 | 879 (= 3.664 × 24) | - |
| 셀 모니터링 IC | LTC6804 × 2 (Slave1, Slave2) | LTC6804 × 1 (Slave3) |
| SOC 알고리즘 | `Farasis52AhSoc` | `Frey60AhSoc` |

상위 제어기(PMS/VCU)와는 **CAN(eCAN-A)** 으로 통신하며, 셀 모니터링 IC와는 **SPI(isoSPI 브리지)** 로 통신한다.

---

## 2. 디렉토리 구조

```
F28069PackBMS/
├── SysSoure/              # 애플리케이션 소스 (※ 폴더명 오타: Source가 아닌 Soure)
│   ├── main.c             # 메인 상태머신, 타이머0 ISR, CAN RX ISR
│   ├── DSP28x_Project.c   # 시스템 핸들러: 전압/전류/온도 계산, Alarm/Fault, CAN, GPIO
│   ├── BAT_LTC6802.c      # LTC6804 셀 모니터링 IC 드라이버 (SPI/PEC/밸런싱/온도)
│   ├── BATAlgorithm.c     # SOC 계산 (Farasis NCM / Frey LFP)
│   └── ProtectRelay.c     # 보호 릴레이(+/−/Pre) 웨이크업 시퀀스
├── SysInclude/            # 애플리케이션 헤더 (파라미터/구조체 정의)
│   ├── parameter.h        # ★ 모든 임계값/상수/핀맵/제품정보 정의
│   ├── BAT_LTC6802.h      # SlaveReg 구조체, LTC6804 명령어 정의
│   ├── BATAlgorithm.h     # SocReg 구조체, SOC 상태 정의
│   ├── ProtectRelay.h     # PrtectRelayReg 구조체
│   └── SysVariable.h      # 공통 include 래퍼
├── C2806Xinclude/
│   └── DSP28x_Project.h   # ★ SystemReg / CANAReg 중앙 데이터 모델 + 비트필드 레지스터
├── C2806XSrc/             # TI F2806x 디바이스 지원 라이브러리 (주변장치 드라이버)
├── targetConfigs/         # CCS 타겟 설정 (TMS320F28069.ccxml)
└── docs/                  # 본 문서
```

> 핵심 데이터 모델(`SystemReg`, `CANAReg`)이 TI 라이브러리 폴더인 `C2806Xinclude/DSP28x_Project.h`에
> 들어 있는 점은 구조상 비정상이다(원래 TI 템플릿 헤더를 확장한 흔적). 향후 `SysInclude`로 이동 권장.

---

## 3. 실행 모델

### 3.1 듀얼 루프 구조

```
                ┌─────────────────────────────────────────────┐
   main()  ──▶  │  while(1) 슈퍼루프 (SysRegs.SysMachine)      │
                │  - 셀 전압/온도 측정 (SPI)                   │
                │  - 셀 밸런싱                                  │
                │  - 상태머신 천이                              │
                └─────────────────────────────────────────────┘
                                  ▲
                                  │ 공유: 전역 SysRegs / CANARegs / SlaveNRegs
                                  ▼
   CPU Timer0 ISR (1ms) ──▶  ┌─────────────────────────────────┐
                             │  - DigitalInput                  │
                             │  - 전류 계산 (80V/12V)           │
                             │  - SOC 적산                       │
                             │  - Alarm/Fault 판정              │
                             │  - CAN 주기 송신 (10/50/100ms)   │
                             │  - DigitalOutput                 │
                             └─────────────────────────────────┘

   eCAN-A RX ISR ──────────▶  외부 전류센서/PMS 명령 수신 (MBOX0~3)
```

- **CPU Timer0**: `PRD = 80400` → 약 1ms 주기. 모든 주기 처리의 시간 기준.
- ISR 내부의 `SysRegTimerNmsecCount` 카운터로 5/10/50/100/300/500/1000ms 슬롯을 분주하여 작업 분산.
- 슈퍼루프는 SPI 측정(블로킹) 위주, ISR은 판정·통신 위주로 역할 분리.

### 3.2 인터럽트 벡터

| 벡터 | 핸들러 | 그룹 | 용도 |
|------|--------|------|------|
| `TINT0` | `cpu_timer0_isr` | INT1.7 | 1ms 메인 주기 |
| `ECAN0INTA` | `ISR_CANRXINTA` | INT9.5 | CAN 수신 |

---

## 4. 시스템 상태머신 (`SysRegs.SysMachine`)

`main.c`의 `while(1)` 안 `switch`로 구현. (`SysState` enum)

```
 INIT ──▶ STANDBY ──▶ READY ──▶ RUNNING
   ▲          │          │   ▲      │
   │          │          ▼   │      ▼
   │          │      PROTECTER ◀────┘   (SysFault 발생 시)
   │          │          │
   └──────────┴──────────┘   (PrtctReset 수신 시 READY 복귀)
```

| 상태 | 역할 |
|------|------|
| `System_STATE_INIT` | 전역 변수/타이머/슬레이브 초기화, SOC 레지스터 초기화 |
| `System_STATE_STANDBY` | 셀 전압/온도 초기 측정, SOC 초기값 산출, 측정 후 READY로 |
| `System_STATE_READY` | CAN 통신 활성, PMS `RUNStatus=1` 수신 시 릴레이 웨이크업 → RUNNING |
| `System_STATE_RUNING` | 정상 운전. 충/방전 진행, 알람 표시 |
| `System_STATE_PROTECTER` | Fault 발생 시 보호 릴레이 차단. `PrtctReset` 수신 시 복구 |
| `DATALOG / ProtectHistory / MANUALMode / CLEAR` | **미구현(빈 case)** — 확장 예약 |

> `System_STATE_ProtectHistory` case가 비어 있다 — 펌웨어 내 보호이력 로깅 기능은 미구현 상태.

---

## 5. 데이터 모델

모든 상태는 소수의 전역 구조체에 집약(전역 공유 방식).

| 전역 변수 | 타입 | 정의 위치 | 역할 |
|-----------|------|-----------|------|
| `SysRegs` | `SystemReg` | DSP28x_Project.h | 팩 단위 측정값/상태/Alarm/Fault/IO 총괄 |
| `CANARegs` | `CANAReg` | DSP28x_Project.h | CAN 송수신용 스케일링 데이터 |
| `Slave1Regs`~`Slave3Regs` | `SlaveReg` | BAT_LTC6802.h | LTC6804 칩별 셀 전압/온도/밸런싱 |
| `Farasis52AhSocRegs` | `SocReg` | BATAlgorithm.h | 80V SOC 적산 상태 |
| `Frey60AhSocRegs` | `SocReg` | BATAlgorithm.h | 12V SOC 적산 상태 |
| `PrtectRelayRegs` | `PrtectRelayReg` | ProtectRelay.h | 보호 릴레이 상태머신 |

### 5.1 비트필드 레지스터 (DSP28x_Project.h)

`union { unsigned all; struct {...} bit; }` 패턴으로 정의:

- `SystemState_REG` — `SysSTATE`(3bit), `SysAalarm`, `SysFault`, `INITOK`, `CANCOMEnable`, 각종 COM_ERR
- `SystemAlarm_REG` — Pack/Cell의 OV/UV/OT/UT/불균형 알람 14종
- `SystemFault_REG` — Alarm 14종 + `CellIR_OV`, `PackRLY_ERR`, `PackISOSPI_Err`, `PackCAN_ERR`, `PackOcTime_Err` 등 (long, 21bit)
- `DigitalInput/Output_REG` — 릴레이 보조접점, CAN RX, LED, IDSW 등 핀 매핑
- `VCUCOMMAND_REG` — 상위제어기 명령(`RUNStatus`, `PrtctReset`)

---

## 6. 주요 모듈

### 6.1 셀 모니터링 — `BAT_LTC6802.c` (LTC6804 드라이버)

> 파일명은 6802지만 실제 명령어 셋은 **LTC6804** (`BAT_LTC6802.h`의 `LTC6804_CMD_*`).

- **SPI 저수준**: `BAT_InitSPI`(마스터, 8bit, ~3.75MHz), `SPI_BATWrite/Read`(폴링)
- **PEC15(CRC-15)**: `init_PEC15_Table`(256엔트리 LUT), `pec15` — 모든 송수신 무결성 검증
- **명령 래퍼**: `LTC6804_write_cmd`(WakeUp→CS→전송), `LTC6804_read_cmd`(수신 후 PEC 검증, valid 반환)
- **셀 전압**: `SlaveVoltagHandler` — ADCV 후 RDCVA/B/C/D로 12셀(4×3) 읽기 → `CellVoltage[]`(mV) → `CellVoltageF[]`(V)
- **온도(NTC)**: `SlaveBMSDigiteldoutOHandler`(GPIO 멀티플렉서로 채널 순환) + `SalveTempsHandler`(ADAX 측정)
  - 3차 다항식 환산: `T = -9.535·V³ + 46.665·V² - 110.33·V + 117.68` (계수 `C_TempsA~DGain`)
- **밸런싱**: `SlaveVoltagBalaHandler`(셀전압 − 최소전압 ≥ 30mV → 비트 ON) → `SlaveBmsBalance`(WRCFG로 DCC 설정)
- **에러카운트**: `ErrorCountA`(전압읽기), `ErrorCountB`(GPIO쓰기), `ErrorCountC`(온도) — 실패 누적, 250 포화, 성공 시 0
  - `ErrorCountA ≥ 250` 시 `PackISOSPI_Err` Fault (main.c ISR)
- **슬레이브 ID**: BMS_ID_1=0xE, BMS_ID_2=0xD, BMS_ID_3=0xC

### 6.2 시스템 계산/판정 — `DSP28x_Project.c`

- **전압**: `Cal80V/12VSysVoltageHandle` — 셀 합산(팩전압), Max/Min/평균/편차
- **전류**: `Cal80V/12VSysCurrentHandle` — 외부 센서 CAN raw(오프셋 0x80000000) → A 환산, 부호(`C_CTDirection`), ±700/±500A 포화, 절댓값
- **온도**: `Cal80V/12VSysTemperatureHandle` — Max/Min/평균/편차
- **Alarm 판정**: `Cal80VSysAlarmtCheck`(히스테리시스 + 100ms 디바운싱), `Cal12VSysAlarmtCheck`(단순 비교)
- **Fault 판정**: `Cal80VSysFaultCheck` — 임계값별 딜레이 카운터로 지연 확정. 버퍼(`BAT80VFaulBuftReg`)/확정(`BAT80VFaultReg`) 분리
- **GPIO**: `DigitalInput`(IDSW, 보조접점, CAN RX), `DigitalOutput`(릴레이, LED 점멸)
- **CAN 송신**: `CANATX(ID, len, d0..d3)` — MBOX31, 송신완료 폴링(타임아웃 2000)

### 6.3 SOC — `BATAlgorithm.c`

- `CalFarasis52Ah*`(NCM), `CalFrey60Ah*`(LFP) 각 3종: RegsInit / SocInit(전압 기반 초기 SOC) / SocHandle(전류 적산)
- 전류 적산(쿨롱 카운팅): `SocCumulativeTime = 1/3600`, 샘플주기 50ms, 셀 용량 52Ah
- LFP는 전압-SOC 구간(`LFP_VOLT_A~D`)으로 평탄 영역 보정
- `SoCStateRegs.bit.CalMeth`: 0=초기값(전압) / 1=적산값 사용

### 6.4 보호 릴레이 — `ProtectRelay.c`

- 상태머신: `STATE_IDLE → WakeUpReady → WakeUpON → WakeUpOFF`
- `ProtectRelayWakeUpHandle` — Pre충전→메인 릴레이 순차 투입 시퀀스(타이밍 `parameter.h`의 `*OnCntVaule`)

---

## 7. CAN 통신 맵 (eCAN-A)

### 7.1 송신 (BMS → PMS/VCU), `CANCOMEnable=1` 조건

| ID | 주기 | 내용 |
|----|------|------|
| 0x600 | 1s/init | SW타입·버전, 정격전압, 용량, Config |
| 0x601 | 10ms | 80V 팩전압, 전류, SOC, SOH |
| 0x602 | 10ms | 80V 상태, 릴레이 출력, Ah, FaultCT |
| 0x603 | 100ms | 80V Alarm, Fault(L/H) |
| 0x604 | 100ms | 80V 충/방전 연속·피크 파워 |
| 0x605 | 100ms | 80V 셀전압 Max/Min/평균/편차 |
| 0x606 | 100ms | 80V 셀온도 Max/Min/평균/편차 |
| 0x607 | 10ms | 12V 팩전압, 전류, SOC, SOH |
| 0x608~0x60B | 100ms | 12V 상태/Alarm·Fault/셀전압/셀온도 |
| 0x60C/0x60D/0x60E | 100ms | 80V 셀별 전압/온도/내부저항 (3셀씩 순차) |
| 0x60F | 100ms | CAN/ISO-SPI 통신 카운터 진단 |

### 7.2 수신 (PMS/VCU → BMS)

| MBOX | ID | 내용 |
|------|-----|------|
| MBOX0 | 0x3C5 | 80V 외부 전류센서 (CAB500 등) |
| MBOX1 | 0x3C2 | 12V 외부 전류센서 |
| MBOX2 | 0x700 | PMS 명령 (`RUNStatus`, `PrtctReset`) |
| MBOX3 | 0x400 | 예약(미사용) |

- CAN 수신 끊김 감시: 100ms 슬롯에서 `SysCanRxCount` 증가, 임계 초과 시 `PackCAN_ERR` + `RUNStatus=0`.

---

## 8. 주요 파라미터 (`parameter.h`)

| 분류 | 매크로 | 값 |
|------|--------|-----|
| 제품 | `Product_Voltage` / `Product_Capacity` | 879 / 45 |
| 제품 | `Product_Type` / `Product_Version` | 3 / 20 |
| 80V Fault | 셀 OV / UV | 4.27V / 2.75V |
| 80V Fault | 팩전류 과류 | 506A |
| 80V Fault | 셀온도 OV / UV | 60℃ / -25℃ |
| 80V 밸런싱 | 시작 편차 / 제한전압 | 30mV / 3.0V |
| 12V Fault | 셀 OV / UV | 3.60V / 2.25V |
| 타이밍 | Timer0 분주 | 5/10/50/100/300/500/1000ms |

---

## 9. 알려진 이슈 / 개선 권장 (코드 리뷰 요약)

> ⚠️ 아래는 정적 분석 기반 **의심 항목**으로, 실제 동작/하드웨어 확인 후 수정 필요.

1. **셀 저전압(80V) Fault 비활성화** — `Cal80VSysFaultCheck`에서 `CellVolt_UN` Fault 설정 라인이 주석 처리됨 → 셀 저전압이 Fault로 보고되지 않을 수 있음.
2. **온도/전압 변수 혼동 의심** — 셀 저온·온도편차 판정에서 전압 변수(`...VoltageF`)로 비교하는 듯한 부분 존재.
3. **Alarm 카운터 인덱스 의심** — `Bat80VAlarmCont[]` 증가 인덱스 불일치 가능성.
4. **폴링 SPI 무한대기 위험** — `SPI_BATWrite/Read`에 타임아웃 없음. 응답 없는 IC에서 행(hang) 가능.
5. **12V 알람 디바운싱 부재** — 80V와 달리 단순 비교 → 노이즈 취약.
6. **CANATX 중복 정의** — 두 곳에 정의됨(통합 권장).
7. **데이터 모델 위치** — `SystemReg`/`CANAReg`가 TI 라이브러리 헤더에 존재(분리 권장).
8. **폴더명 오타** — `SysSoure`(→ Source).

상세 내용은 코드 리뷰 시 항목별 라인 확인 필요.

---

## 10. 빌드 / 인코딩 메모

- **인코딩**: 전체 소스는 **UTF-8(BOM 없음)**. Eclipse 프로젝트 기본값도 `encoding/<project>=UTF-8`.
  (과거 CP949 저장본은 2026-06-02 UTF-8로 일괄 변환됨. 한글은 전부 주석에만 존재하여 컴파일 무영향.)
- **빌드 산출물**: `Debug/`, `*.out`, `*.map` 등은 `.gitignore`로 제외.
- 변경 이력은 [변경이력.md](변경이력.md) 참조.
