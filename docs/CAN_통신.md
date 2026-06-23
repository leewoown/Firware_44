# CAN 통신

> 30kW 수소지게차 80V PACK BMS — CAN 통신 세션
> **기준**: [20260610_30kW_제품규격서R16_통신규약.md](20260610_30kW_제품규격서R16_통신규약.md) (제품규격서 R16)
> **버스**: eCAN-A, 표준 11-bit ID, DLC=8 / **송신** MBOX31, **수신** MBOX0~3

---

## 1. 개요

| 항목 | 내용 |
|------|------|
| 컨트롤러 | eCAN-A (TMS320F28069 내장) |
| 프레임 | 표준 11-bit ID, Classic CAN |
| 송신 메일박스 | MBOX31 (`CANATX`) |
| 수신 메일박스 | MBOX0 (0x3C5), MBOX1 (0x3C2), MBOX2 (0x700), MBOX3 (0x400 예약) |
| 총 정의 메시지 | 19개 (TX 15 + RX 4) |
| 총 시그널 | 130개 |

상위 제어기(IFCU/PMS/VCU)와 BMS(BSA) 간 상태·보호 정보를 교환하고,
외부 전류센서(HV_CT / LV_CT)로부터 전류 측정값을 수신한다.

---

## 2. 메시지 맵

### 2.1 수신 (→ BMS)

| MBOX | CAN ID | 이름 | ECU | 주기 | 내용 |
|:----:|:------:|------|:---:|:----:|------|
| MBOX0 | 0x3C5 | HV_CT | HV_CT | 10ms | 80V 고전압 전류센서 (CSNV700) |
| MBOX1 | 0x3C2 | CAB500_IP | LV_CT | 10ms | 12V 저전압 전류센서 (CAB500-C/SP5) |
| MBOX2 | 0x700 | IFCU_BSA | IFCU | 100ms | 운전/리셋 명령 |
| MBOX3 | 0x400 | (예약) | - | - | 미사용 |

### 2.2 송신 (BMS →)

| CAN ID | 이름 | 주기 | 내용 |
|:------:|------|:----:|------|
| 0x600 | BSA1 | 1000ms | 제품정보 (Type/Ver/공칭전압/용량/구성) |
| 0x601 | BSA2 | 10ms | 80V 전압/전류/SOC/SOH |
| 0x602 | BSA3 | 10ms | 상태/밸런스/릴레이/Ah |
| 0x603 | BSA4 | 100ms | 경고(Warning)/보호(Protection) 비트 |
| 0x604 | BSA5 | 100ms | 충/방전 연속·피크 파워제한 |
| 0x605 | BSA6 | 100ms | 셀전압 Max/Min/Avg/편차 |
| 0x606 | BSA7 | 100ms | 셀온도 Max/Min/Avg/편차 |
| 0x607 | BSA8 | 10ms | 보조(12V) 전압/전류/SOC/SOH |
| 0x608 | BSA9 | 100ms | 보조 상태/Ah |
| 0x609 | BSA10 | 100ms | 보조 경고/보호 |
| 0x60A | BSA11 | 100ms | 보조 셀전압 Max/Min/Avg/편차 |
| 0x60B | BSA12 | 100ms | 보조 셀온도 Max/Min/Avg/편차 |
| 0x60C | BSA13 | 100ms | 셀전압 개별 (Mux) |
| 0x60D | BSA14 | 100ms | 셀온도 개별 |
| 0x60E | BSA15 | 100ms | 셀 내부저항(IR) |
| 0x60F | BSA16 | 100ms | 통신 카운터 진단 |

---

## 3. 수신 메시지 상세

### 3.1 HV_CT (0x3C5) — 80V 고전압 전류센서 (CSNV700)

| Signal | Len | Bit | Type | Factor | Offset | 단위 | 설명 |
|--------|:---:|:---:|:----:|:------:|:------:|:----:|------|
| IP_VALUE | 32 | 24 | U | 0.001 | -2147483.648 | A | 전류 측정값. `Phys = Raw×0.001 + (-2147483.648)`, ±700A |
| ERROR_INFORMATION | 7 | 32 | U | 1 | | | 진단 에러코드(7bit) |
| ERROR_INDICATION | 1 | 39 | U | 1 | | flag | 0=정상, 1=장애(IP_VALUE 무효) |
| VACANT_DATA_2BYTES | 16 | 48 | U | 1 | | | 예약(Fixed 0) |
| CRC_8 | 8 | 56 | U | 1 | | | CRC-8 (POLY x⁸+x²+x+1, Byte0~6 대상) |

### 3.2 CAB500_IP (0x3C2) — 12V 저전압 전류센서 (CAB500-C/SP5)

| Signal | Len | Bit | Type | Factor | Offset | 단위 | 설명 |
|--------|:---:|:---:|:----:|:------:|:------:|:----:|------|
| IP_VALUE | 32 | 0 | U | 0.001 | -2147483.648 | A | 전류 측정값. ±500A |
| ERROR_INDICATION | 1 | 32 | U | 1 | | flag | 0=정상, 1=장애 |
| ERROR_INFO | 7 | 33 | U | 1 | | | CSM-FAIL 진단코드(7bit) |
| SENSOR_NAME | 24 | 40 | U | 1 | | | 센서 식별명 'CAB500' (ASCII) |

### 3.3 IFCU_BSA (0x700) — 운전/리셋 명령

| Signal | Len | Bit | Order | 단위 | 설명 |
|--------|:---:|:---:|:-----:|:----:|------|
| Ifcu_BSARun | 1 | 8 | LSB | flag | 0:BSA OFF, 1:BSA ON |
| Ifcu_BSAReset | 1 | 9 | LSB | flag | 0:No Action, 1:Fault Reset |

> 수신 시 CAN 타임아웃 감시 카운터(`SysCanRxCount`) 리셋.

---

## 4. 송신 메시지 상세

### 4.1 BSA1 (0x600) — 제품정보 / 1000ms

| Signal | Len | Bit | Init | Factor | 단위 | 설명 |
|--------|:---:|:---:|:----:|:------:|:----:|------|
| BSA_Type | 8 | 0 | 3 | 1 | | 0:EV BUS, 1:Ship, 2:Subway, 3:E-Mobility |
| BSA_SW_Ver | 8 | 8 | | 0.1 | VER | Software Version |
| BSA_Normal_Volt | 16 | 16 | 87.9 | 0.1 | V | 공칭전압 |
| BSA_Capacity | 16 | 32 | 52 | 0.1 | Ah | 팩 용량 |
| BSA_Serial | 8 | 48 | 23 | 1 | S | 직렬 셀 수 |
| BSA_Parallel | 8 | 56 | 1 | 1 | P | 병렬 셀 수 |

### 4.2 BSA2 (0x601) — 80V 전압/전류/SOC/SOH / 10ms

| Signal | Len | Bit | Type | Factor | 단위 | Min | Max |
|--------|:---:|:---:|:----:|:------:|:----:|:---:|:---:|
| BSA_Volt_Total | 16 | 0 | U | 0.1 | V | 40 | 120 |
| BSA_Curr_Total | 16 | 16 | S | 0.1 | A | -700 | 700 |
| BSA_SOC | 16 | 32 | U | 0.1 | % | 0 | 120 |
| BSA_SOH | 16 | 48 | U | 0.1 | % | 0 | 100 |

### 4.3 BSA3 (0x602) — 상태/릴레이/Ah / 10ms

| Signal | Len | Bit | 설명 |
|--------|:---:|:---:|------|
| BSA_Ste | 3 | 0 | 0:Init, 1:Ready, 2:Run, 3:… (BMS State) |
| BSA_Balance | 1 | 3 | 0:Balance On, 1:Off |
| BSA_Neg_Rly | 1 | 16 | 0:Open, 1:Close |
| BSA_Pos_Rly | 1 | 17 | 0:Open, 1:Close |
| BSA_PreChar_Rly | 1 | 18 | 0:Open, 1:Close |
| BSA_Ah | 16 | 32 | Signed, 0.1 Ah (디버깅용) |

### 4.4 BSA4 (0x603) — 경고/보호 / 100ms ★

→ 비트 구조는 [6. BSA4 Fault Bit 구조](#6-bsa4-0x603-fault-bit-구조) 참조.

### 4.5 BSA5 (0x604) — 파워제한 / 100ms

| Signal | Len | Bit | Init | Factor | 단위 | 설명 |
|--------|:---:|:---:|:----:|:------:|:----:|------|
| BSA_Charge_Cont_PL | 16 | 0 | 2.5 | 0.1 | kW | 충전 연속 제한 |
| BSA_Discharge_Cont_PL | 16 | 16 | 2.5 | 0.1 | kW | 방전 연속 제한 |
| BSA_Charge_Peak_PL | 16 | 32 | 2.5 | 0.1 | kW | 충전 피크 제한 |
| BSA_Discharge_Peak_PL | 16 | 48 | 2.5 | 0.1 | kW | 방전 피크 제한 |

### 4.6 BSA6 (0x605) — 셀전압 통계 / 100ms

| Signal | Len | Bit | Factor | 단위 | 설명 |
|--------|:---:|:---:|:------:|:----:|------|
| BSA_Cell_MaxV | 16 | 0 | 0.001 | V | 최대 셀전압 |
| BSA_Cell_MinV | 16 | 16 | 0.001 | V | 최저 셀전압 |
| BSA_Cell_AVGV | 16 | 32 | 0.001 | V | 평균 셀전압 |
| BSA_Cell_DeviV | 16 | 48 | 1 | mV | 셀전압 편차 |

### 4.7 BSA7 (0x606) — 셀온도 통계 / 100ms

| Signal | Len | Bit | Type | Factor | 단위 | 설명 |
|--------|:---:|:---:|:----:|:------:|:----:|------|
| BSA_Cell_MaxT | 16 | 0 | S | 0.1 | ℃ | 최대 셀온도 |
| BSA_Cell_MinT | 16 | 16 | S | 0.1 | ℃ | 최저 셀온도 |
| BSA_Cell_AVGT | 16 | 32 | S | 0.1 | ℃ | 평균 셀온도 |
| BSA_Cell_DeviT | 16 | 48 | U | 0.1 | ℃ | 셀온도 편차 |

### 4.8 BSA8 (0x607) — 보조(12V) 전압/전류/SOC/SOH / 10ms

| Signal | Len | Bit | Type | Factor | 단위 |
|--------|:---:|:---:|:----:|:------:|:----:|
| BSA_Volt_Start_Total | 16 | 0 | U | 0.1 | V |
| BSA_Curr_Start_Total | 16 | 16 | S | 0.1 | A |
| BSA_SOC_Start | 16 | 32 | U | 0.1 | % |
| BSA_SOH_Start | 16 | 48 | U | 0.1 | % |

### 4.9 BSA9 (0x608) — 보조 상태/Ah / 100ms

| Signal | Len | Bit | 설명 |
|--------|:---:|:---:|------|
| BSA_Start_Balance | 1 | 0 | 0:Balance On, 1:Off |
| BSA_Start_Pos_Rly | 1 | 1 | 0:Open, 1:Close |
| BSA_Start_Ah | 16 | 32 | Signed, 0.1 Ah (디버깅용) |

### 4.10 BSA10 (0x609) — 보조 경고/보호 / 100ms

→ Warning(bit 0~13) / Protection(bit 16~30) 구조. 80V BSA4와 동일 패턴이며 `BSA_Start_*` 접두어 사용.

### 4.11 BSA11 (0x60A) — 보조 셀전압 통계 / 100ms

| Signal | Len | Bit | Factor | 단위 |
|--------|:---:|:---:|:------:|:----:|
| BSA_Start_Cell_MaxV | 16 | 0 | 0.001 | V |
| BSA_Start_Cell_MinV | 16 | 16 | 0.001 | V |
| BSA_Start_Cell_AVGV | 16 | 32 | 0.001 | V |
| BSA_Start_Cell_DeviV | 16 | 48 | 1 | mV |

### 4.12 BSA12 (0x60B) — 보조 셀온도 통계 / 100ms

| Signal | Len | Bit | Type | Factor | 단위 |
|--------|:---:|:---:|:----:|:------:|:----:|
| BSA_Start_Cell_MaxT | 16 | 0 | S | 0.1 | ℃ |
| BSA_Start_Cell_MinT | 16 | 16 | S | 0.1 | ℃ |
| BSA_Start_Cell_AVGT | 16 | 32 | S | 0.1 | ℃ |
| BSA_Start_Cell_DeviT | 16 | 48 | U | 0.1 | ℃ |

### 4.13 BSA13 (0x60C) — 셀전압 개별 (Mux) / 100ms

| Signal | Len | Bit | Mux | 설명 |
|--------|:---:|:---:|:---:|------|
| BMS_80VCELLV_Num | 16 | 0 | M | 시작 셀번호 (멀티플렉서) |
| BMS_80VCell_Vlolt | 16 | 16 | M | 셀전압 (0.001 V) |
| BMS_80VCell_Temps | 16 | 32 | m0 | 셀온도 |
| BMS_80VCell_IRNum2 | 16 | 48 | m0 | 셀 내부저항 |

### 4.14 BSA14 (0x60D) — 셀온도 개별 / 100ms

| Signal | Len | Bit | 설명 |
|--------|:---:|:---:|------|
| BMS_DUB1 | 64 | 0 | 셀온도 개별 데이터 |

### 4.15 BSA15 (0x60E) — 셀 내부저항(IR) / 100ms

| Signal | Len | Bit | 설명 |
|--------|:---:|:---:|------|
| BMS_DUB2 | 64 | 0 | 셀 내부저항 데이터 |

### 4.16 BSA16 (0x60F) — 통신 카운터 진단 / 100ms

| Signal | Len | Bit | 단위 | 설명 |
|--------|:---:|:---:|:----:|------|
| BMS_CANRxCount | 8 | 0 | Cnt | 전체 CAN Rx 카운터 |
| BMS_80VCT_RxCount | 8 | 8 | Cnt | 80V 전류센서 Rx |
| BMS_12VCT_RxCount | 8 | 16 | Cnt | 12V 전류센서 Rx |
| BMS_VCU_RxCount | 8 | 24 | Cnt | VCU Rx |
| BMS_VCU_SetCount | 16 | 32 | Cnt | VCU Set |
| BMS_SlaveNum | 8 | 48 | Cnt | 슬레이브 수 |
| BMS_ISOSPIErrCnt | 8 | 56 | Cnt | ISO-SPI 에러 카운터 |

---

## 5. 전류센서 에러코드

### 5.1 CSNV700 (0x3C5) — Honeywell 고전압 전류센서

| 에러코드 | 설명 |
|:--------:|------|
| 0x48 | Flash CRC Error |
| 0x49 | AFE Over Range (Ip > 740A) |
| 0x50 | AFE Error |
| 0x51 | Internal LUT Error |
| 0x54 | Power Minimum Limit (Vs 저전압) |
| 0x55 | Power Maximum Limit (Vs 과전압) |

> 장애 시 `IP_VALUE = 0xFFFFFFFF`, `ERROR_INDICATION = 1`
> 물리값 변환: `Physical(A) = Raw × 0.001 + (-2147483.648)`

### 5.2 CAB500-C/SP5 (0x3C2) — LEM 저전압 전류센서

| 항목 | 설명 |
|------|------|
| ERROR_INDICATION | 0=정상, 1=장애 (IP_VALUE 무효) |
| ERROR_INFO | CSM-FAIL 진단코드 (7bit) |
| SENSOR_NAME | 센서 식별명 'CAB500' (24bit ASCII) |

> 물리값 변환: `Physical(A) = Raw × 0.001 + (-2147483.648)`

---

## 6. BSA4 (0x603) Fault Bit 구조

### 6.1 Warning (bit 0~14)

| Bit | Signal | 설명 |
|:---:|--------|------|
| 0 | PackOC | 과전류 경고 |
| 1 | PackVSOC_OV | SOC 과충전 경고 |
| 2 | PackVSOC_UN | SOC 과방전 경고 |
| 3 | PackVolt_OV | 팩 과전압 경고 |
| 4 | PackVolt_UN | 팩 저전압 경고 |
| 5 | PackTemp_OV | 팩 과온도 경고 |
| 6 | PackTemp_UN | 팩 저온도 경고 |
| 7 | PackUnPWR_BL | 전력 불균형 경고 |
| 8 | CellVolt_OV | 셀 과전압 경고 |
| 9 | CellVolt_UN | 셀 저전압 경고 |
| 10 | CellVolt_BL | 셀 전압 편차 경고 |
| 11 | CellTemp_OV | 셀 과온도 경고 |
| 12 | CellTemp_UN | 셀 저온도 경고 |
| 13 | CellTemp_BL | 셀 온도 편차 경고 |
| 14 | PackCanTmOut | 팩 통신 에러 (CAN 타임아웃) |

### 6.2 Protection (bit 16~34)

| Bit | Signal | 설명 |
|:---:|--------|------|
| 16 | Bsa_PrtctOc | 과전류 보호 |
| 17 | Bsa_PrtctSocH | SOC 과충전 보호 |
| 18 | Bsa_PrtctSocL | SOC 과방전 보호 |
| 19 | Bsa_PrtctOv | 과전압 보호 |
| 20 | Bsa_PrtctUv | 저전압 보호 |
| 21 | Bsa_PrtctOt | 과온도 보호 |
| 22 | Bsa_PrtctUt | 저온도 보호 |
| 23 | Bsa_PrtctUnbalPwr | 전력 불균형 보호 |
| 24 | Bsa_PrtctCellOv | 셀 과전압 보호 |
| 25 | Bsa_PrtctCellUv | 셀 저전압 보호 |
| 26 | Bsa_PrtctCellUnbalVlt | 셀 전압 편차 보호 |
| 27 | Bsa_PrtctCellOt | 셀 과온도 보호 |
| 28 | Bsa_PrtctCellUt | 셀 저온도 보호 |
| 29 | Bsa_PrtctCellUnbalTmp | 셀 온도 편차 보호 |
| 30 | Bsa_FltRly | 릴레이 장애 보호 |
| 31 | Bsa_PrtctCanTmOut | CAN 타임아웃 보호 |
| 32 | Bsa_PrtctCellIr | 셀 IR 과대 보호 |
| 33 | Bsa_PrtctOcTm | OC 타이머 보호 (전류 유지시간 차단) |
| 34 | Bsa_PrtctOcCnt | OC 카운트 보호 (1분 2회 차단) |

---

> 본 문서는 제품규격서 R16을 기준으로 작성한 CAN 통신 세션이다.
> 펌웨어 실제 구현과의 차이는 [통신규약_소스기준.md](통신규약_소스기준.md) 및
> [ARCHITECTURE.md](ARCHITECTURE.md) §7(CAN 통신 맵)을 함께 참조한다.
