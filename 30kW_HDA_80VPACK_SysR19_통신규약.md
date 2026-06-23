# 30kW 수소지게차 80V PACK CAN 통신규약
**DBC: HDA_80VPACK_SysR19**

## 메시지 요약

| No | ECU | Message | CAN ID | DLC | Cycle(ms) | 시그널 수 | Sendable | Receivable |
|:--:|:---:|:-------:|:------:|:---:|:---------:|:---------:|:--------:|:----------:|
| 1 | HV_CT | HV_CT | 0x3C5 | 8 | 10 | 3 | None | None |
| 2 | BSA | BSA1 | 0x600 | 8 | 1000 | 6 | False | True |
| 3 | BSA | BSA2 | 0x601 | 8 | 10 | 4 | False | True |
| 4 | BSA | BSA3 | 0x602 | 8 | 100 | 7 | True | True |
| 5 | BSA | BSA4 | 0x603 | 8 | 100 | 33 | True | True |
| 6 | BSA | BSA5 | 0x604 | 8 | 100 | 4 | True | True |
| 7 | BSA | BSA6 | 0x605 | 8 | 100 | 4 | True | True |
| 8 | BSA | BSA7 | 0x606 | 8 | 100 | 4 | True | True |
| 9 | BSA | BSA8 | 0x607 | 8 | 100 | 4 | True | True |
| 10 | BSA | BSA9 | 0x608 | 8 | 100 | 2 | True | True |
| 11 | BSA | BSA10 | 0x609 | 8 | 100 | 29 | True | True |
| 12 | BSA | BSA11 | 0x60A | 8 | 100 | 4 | True | True |
| 13 | BSA | BSA12 | 0x60B | 8 | 100 | 4 | True | True |
| 14 | BSA | BAS13 | 0x60C | 8 | 100 | 4 | True | True |
| 15 | BSA | BAS14 | 0x60D | 8 | 100 | 4 | False | True |
| 16 | BSA | BAS16 | 0x60F | 8 | 100 | 7 | None | None |
| 17 | IFCU | IFCU_BSA | 0x700 | 8 | 100 | 2 | True | True |

---

## 시그널 상세

### HV_CT (0x3C5) — HV_CT, DLC=8, Cycle=10ms

| Signal | Length | Start Bit | Byte Order | Value Type | Factor | Offset | Min | Max | Unit |
|:-------|:------:|:---------:|:----------:|:----------:|:------:|:------:|:---:|:---:|:----:|
| HV_CT_DATA | 32 | 0 | LSB | Signed | 1 | 0 | 0 | 4294967296 | D |
| HV_CT_Infro | 7 | 32 | LSB | Unsigned | 1 | 0 | 0 | 0 |  |
| HV_CT_ERR | 1 | 39 | LSB | Unsigned | 1 | 0 | 0 | 0 | F |

### BSA1 (0x600) — BSA, DLC=8, Cycle=1000ms

| Signal | Length | Start Bit | Byte Order | Value Type | Factor | Offset | Min | Max | Unit |
|:-------|:------:|:---------:|:----------:|:----------:|:------:|:------:|:---:|:---:|:----:|
| BSA_SW_Ver | 8 | 0 | LSB | Unsigned | 1 | 0 | 0 | 255 | SWVer |
| BSA_Type | 8 | 8 | LSB | Unsigned | 1 | 0 | 0 | 255 | ty |
| BSA_Normal_Volt | 16 | 16 | LSB | Unsigned | 1 | 0 | 0 | 65535 | V |
| BSA_Capacity | 16 | 32 | LSB | Unsigned | 0.1 | 0 | 0 | 6553.5 | Ah |
| BSA_Serial | 8 | 48 | LSB | Unsigned | 1 | 0 | 0 | 255 | s |
| BSA_Parallel | 8 | 56 | LSB | Unsigned | 1 | 0 | 0 | 255 | P |

### BSA2 (0x601) — BSA, DLC=8, Cycle=10ms

| Signal | Length | Start Bit | Byte Order | Value Type | Factor | Offset | Min | Max | Unit |
|:-------|:------:|:---------:|:----------:|:----------:|:------:|:------:|:---:|:---:|:----:|
| BSA_Volt_Total | 16 | 0 | LSB | Unsigned | 0.1 | 0 | 0 | 150 | type |
| BSA_Curr_Total | 16 | 16 | LSB | Signed | 0.1 | 0 | -500 | 500 | A |
| BSA_SOC | 16 | 32 | LSB | Unsigned | 0.1 | 0 | 0 | 100 | % |
| BSA_SOH | 16 | 48 | LSB | Unsigned | 0.1 | 0 | 0 | 100 | % |

### BSA3 (0x602) — BSA, DLC=8, Cycle=100ms

| Signal | Length | Start Bit | Byte Order | Value Type | Factor | Offset | Min | Max | Unit |
|:-------|:------:|:---------:|:----------:|:----------:|:------:|:------:|:---:|:---:|:----:|
| BSA_Ste | 3 | 0 | LSB | Unsigned | 1 | 0 | 0 | 7 | F |
| BSA_Balance | 1 | 3 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Neg_Rly | 1 | 16 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Pos_Rly | 1 | 17 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_PreChar_Rly | 1 | 18 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Ah | 16 | 32 | LSB | Signed | 0.1 | 0 | -100 | -100 | % |
| BSA_Curr_Fault_Total | 16 | 48 | LSB | Signed | 0.1 | 0 | -700 | 700 | A |

### BSA4 (0x603) — BSA, DLC=8, Cycle=100ms

| Signal | Length | Start Bit | Byte Order | Value Type | Factor | Offset | Min | Max | Unit |
|:-------|:------:|:---------:|:----------:|:----------:|:------:|:------:|:---:|:---:|:----:|
| BSA_Wn_OC | 1 | 0 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Wn_SOC_OV | 1 | 1 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Wn_SOC_Un | 1 | 2 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Wn_OV | 1 | 3 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Wn_UV | 1 | 4 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Wn_OT | 1 | 5 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Wn_UT | 1 | 6 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Wn_UnbalancePower | 1 | 7 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Wn_Cell_UV | 1 | 8 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Wn_Cell_OV | 1 | 9 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Wn_Cell_UnbalV | 1 | 10 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Wn_Cell_OT | 1 | 11 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Wn_Cell_UT | 1 | 12 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Wn_Cell_UnbalT | 1 | 13 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Prtct_OC | 1 | 16 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Prtct_SOC_OV | 1 | 17 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Prtct_SOC_Un | 1 | 18 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Prtct_OV | 1 | 19 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Prtct_UV | 1 | 20 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Prtct_OT | 1 | 21 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Prtct_UT | 1 | 22 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Prtct_UnbalancePower | 1 | 23 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Prtct_Cell_OV | 1 | 24 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Prtct_Cell_UV | 1 | 25 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Prtct_Cell_UnbalV | 1 | 26 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Prtct_Cell_OT | 1 | 27 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Prtct_Cell_UT | 1 | 28 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Prtct_Cell_UnbalT | 1 | 29 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Prtct_Rly_Err | 1 | 30 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| Bsa_PrtctCanTmOut | 1 | 31 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| Bsa_Prtct_CellIR_OV | 1 | 32 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| Bsa_PrtcOcTimer | 1 | 33 | LSB | Unsigned | 1 | 0 | 0 | 1 |  |
| Bsa_PrtcOcTime_min | 1 | 34 | LSB | Unsigned | 1 | 0 | 0 | 1 |  |

### BSA5 (0x604) — BSA, DLC=8, Cycle=100ms

| Signal | Length | Start Bit | Byte Order | Value Type | Factor | Offset | Min | Max | Unit |
|:-------|:------:|:---------:|:----------:|:----------:|:------:|:------:|:---:|:---:|:----:|
| BSA_Charge_Cont_PL | 16 | 0 | LSB | Unsigned | 0.1 | 0 | 0 | 20 | kW |
| BSA_Discharge_Cont_PL | 16 | 16 | LSB | Unsigned | 1 | 0 | 0 | 20 | kW |
| BSA_Charge_Peak_PL | 16 | 32 | LSB | Unsigned | 1 | 0 | 0 | 20 | kW |
| BSA_Discharge_Peak_PL | 16 | 48 | LSB | Unsigned | 1 | 0 | 0 | 20 | kW |

### BSA6 (0x605) — BSA, DLC=8, Cycle=100ms

| Signal | Length | Start Bit | Byte Order | Value Type | Factor | Offset | Min | Max | Unit |
|:-------|:------:|:---------:|:----------:|:----------:|:------:|:------:|:---:|:---:|:----:|
| BSA_Cell_MaxV | 16 | 0 | LSB | Unsigned | 0.001 | 0 | 0 | 4.5 | V |
| BSA_Cell_MinV | 16 | 16 | LSB | Unsigned | 0.001 | 0 | 0 | 4.5 | V |
| BSA_Cell_AVGV | 16 | 32 | LSB | Unsigned | 0.001 | 0 | 0 | 4.5 | V |
| BSA_Cell_DeviV | 16 | 48 | LSB | Unsigned | 1 | 0 | 0 | 4.5 | mV |

### BSA7 (0x606) — BSA, DLC=8, Cycle=100ms

| Signal | Length | Start Bit | Byte Order | Value Type | Factor | Offset | Min | Max | Unit |
|:-------|:------:|:---------:|:----------:|:----------:|:------:|:------:|:---:|:---:|:----:|
| BSA_Cell_MaxT | 16 | 0 | LSB | Signed | 0.1 | 0 | -30 | 100 | deg |
| BSA_Cell_MinT | 16 | 16 | LSB | Signed | 0.1 | 0 | -30 | 100 | deg |
| BSA_Cell_AVGT | 16 | 32 | LSB | Signed | 0.1 | 0 | -30 | 100 | deg |
| BSA_Cell_DeviT | 16 | 48 | LSB | Unsigned | 0.1 | 0 | 0 | 15 | deg |

### BSA8 (0x607) — BSA, DLC=8, Cycle=100ms

| Signal | Length | Start Bit | Byte Order | Value Type | Factor | Offset | Min | Max | Unit |
|:-------|:------:|:---------:|:----------:|:----------:|:------:|:------:|:---:|:---:|:----:|
| BSA_Volt_Start_Total | 16 | 0 | LSB | Unsigned | 0.1 | 0 | 0 | 20 | type |
| BSA_Curr_Start_Total | 16 | 16 | LSB | Signed | 0.1 | 0 | -500 | 500 | A |
| BSA_SOC_Start | 16 | 32 | LSB | Unsigned | 0.1 | 0 | 0 | 100 | % |
| BSA_SOH_Start | 16 | 48 | LSB | Unsigned | 0.1 | 0 | 0 | 100 | % |

### BSA9 (0x608) — BSA, DLC=8, Cycle=100ms

| Signal | Length | Start Bit | Byte Order | Value Type | Factor | Offset | Min | Max | Unit |
|:-------|:------:|:---------:|:----------:|:----------:|:------:|:------:|:---:|:---:|:----:|
| BSA_Start_Balance | 1 | 0 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Pos_Rly | 1 | 1 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |

### BSA10 (0x609) — BSA, DLC=8, Cycle=100ms

| Signal | Length | Start Bit | Byte Order | Value Type | Factor | Offset | Min | Max | Unit |
|:-------|:------:|:---------:|:----------:|:----------:|:------:|:------:|:---:|:---:|:----:|
| BSA_Start_Wn_OC | 1 | 0 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Wn_SOC_OV | 1 | 1 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Wn_SOC_Un | 1 | 2 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Wn_OV | 1 | 3 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Wn_UV | 1 | 4 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Wn_OT | 1 | 5 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Wn_UT | 1 | 6 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Wn_UnbalancePower | 1 | 7 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Wn_Cell_OV | 1 | 8 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Wn_Cell_UV | 1 | 9 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Wn_Cell_UnbalV_Start | 1 | 10 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Wn_Cell_OT_Start | 1 | 11 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Wn_Cell_UT_Start | 1 | 12 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Wn_Cell_UnbalT_Start | 1 | 13 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Prtct_OC_Start | 1 | 16 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Prtct_SOC_OV_Start | 1 | 17 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Prtct_SOC_Un_Start | 1 | 18 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Prtct_OV_Start | 1 | 19 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Prtct_UV | 1 | 20 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Prtct_OT | 1 | 21 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Prtct_UT | 1 | 22 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Prtct_UnbalancePower | 1 | 23 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Prtct_Cell_OV | 1 | 24 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Prtct_Cell_UV | 1 | 25 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Prtct_Cell_UnbalV | 1 | 26 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Prtct_Cell_OT | 1 | 27 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Prtct_Cell_UT | 1 | 28 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Prtct_Cell_UnbalT | 1 | 29 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| BSA_Start_Prtct_Rly_Err | 1 | 30 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |

### BSA11 (0x60A) — BSA, DLC=8, Cycle=100ms

| Signal | Length | Start Bit | Byte Order | Value Type | Factor | Offset | Min | Max | Unit |
|:-------|:------:|:---------:|:----------:|:----------:|:------:|:------:|:---:|:---:|:----:|
| BSA_Start_Cell_MaxV | 16 | 0 | LSB | Unsigned | 0.001 | 0 | 0 | 4.5 | V |
| BSA_Start_Cell_MinV | 16 | 16 | LSB | Unsigned | 0.001 | 0 | 0 | 4.5 | V |
| BSA_Start_Cell_AVGV | 16 | 32 | LSB | Unsigned | 0.001 | 0 | 0 | 4.5 | V |
| BSA_Start_Cell_DeviV | 16 | 48 | LSB | Unsigned | 1 | 0 | 0 | 1000 | mV |

### BSA12 (0x60B) — BSA, DLC=8, Cycle=100ms

| Signal | Length | Start Bit | Byte Order | Value Type | Factor | Offset | Min | Max | Unit |
|:-------|:------:|:---------:|:----------:|:----------:|:------:|:------:|:---:|:---:|:----:|
| BSA_Start_Cell_MaxT | 16 | 0 | LSB | Signed | 0.1 | 0 | -30 | 100 | deg |
| BSA_Start_Cell_MinT | 16 | 16 | LSB | Signed | 0.1 | 0 | -30 | 100 | deg |
| BSA_Start_Cell_AVGT | 16 | 32 | LSB | Signed | 0.1 | 0 | -30 | 100 | deg |
| BSA_Start_Cell_DeviT | 16 | 48 | LSB | Signed | 0.1 | 0 | -30 | 100 | deg |

### BAS13 (0x60C) — BSA, DLC=8, Cycle=100ms

| Signal | Length | Start Bit | Byte Order | Value Type | Factor | Offset | Min | Max | Unit |
|:-------|:------:|:---------:|:----------:|:----------:|:------:|:------:|:---:|:---:|:----:|
| BMS_CELLV_Num | 16 | 0 | LSB | Unsigned | 1 | 0 | 0 | 30 | Num |
| BMS_Cell_VloltNum | 16 | 16 | LSB | Unsigned | 0.001 | 0 | 0 | 4.5 | V |
| BMS_Cell_VloltNum1 | 16 | 32 | LSB | Unsigned | 0.001 | 0 | 0 | 4.5 | V |
| BMS_Cell_VloltNum2 | 16 | 48 | LSB | Unsigned | 0.001 | 0 | 0 | 4.5 | V |

### BAS14 (0x60D) — BSA, DLC=8, Cycle=100ms

| Signal | Length | Start Bit | Byte Order | Value Type | Factor | Offset | Min | Max | Unit |
|:-------|:------:|:---------:|:----------:|:----------:|:------:|:------:|:---:|:---:|:----:|
| BMS_CELLT_Num | 16 | 0 | LSB | Unsigned | 1 | 0 | 0 | 30 | Num |
| BMS_Cell_TempsNum | 16 | 16 | LSB | Signed | 0.1 | 0 | -30 | 100 | deg |
| BMS_Cell_TempsNum1 | 16 | 32 | LSB | Signed | 0.1 | 0 | -30 | 100 | deg |
| BMS_Cell_TempsNum2 | 16 | 48 | LSB | Signed | 0.1 | 0 | -30 | 100 | deg |

### BAS16 (0x60F) — BSA, DLC=8, Cycle=100ms

| Signal | Length | Start Bit | Byte Order | Value Type | Factor | Offset | Min | Max | Unit |
|:-------|:------:|:---------:|:----------:|:----------:|:------:|:------:|:---:|:---:|:----:|
| BMS_CANRxCount | 8 | 0 | LSB | Unsigned | 1 | 0 | 0 | 200 | Cnt |
| BMS_80VCT_RxCount | 8 | 8 | LSB | Unsigned | 1 | 0 | 0 | 200 | Cnt |
| BMS_12VCT_RxCount | 8 | 16 | LSB | Unsigned | 1 | 0 | 0 | 200 | Cnt |
| BMS_VCU_RxCount | 8 | 24 | LSB | Unsigned | 1 | 0 | 0 | 200 | Cnt |
| BMS_VCU_SetCount | 16 | 32 | LSB | Signed | 1 | 0 | 0 | 0 |  |
| BMS_SlaveNum | 8 | 48 | LSB | Unsigned | 1 | 0 | 0 | 20 | Num |
| BMS_ISOSPIErrCnt | 8 | 56 | LSB | Unsigned | 1 | 0 | 0 | 255 | Cnt |

### IFCU_BSA (0x700) — IFCU, DLC=8, Cycle=100ms

| Signal | Length | Start Bit | Byte Order | Value Type | Factor | Offset | Min | Max | Unit |
|:-------|:------:|:---------:|:----------:|:----------:|:------:|:------:|:---:|:---:|:----:|
| Ifcu_BSARun | 1 | 8 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |
| Ifcu_BSAReset | 1 | 9 | LSB | Unsigned | 1 | 0 | 0 | 1 | F |

---
**총 메시지: 17개 / 총 시그널: 125개**
