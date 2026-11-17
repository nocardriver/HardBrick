# MX-01P锛圔LE 5.3 閫忎紶锛塀SP 妯″潡璁捐鏂囨。

## 1. 妯″潡姒傝堪

鏈ā鍧楁彁渚涘浜鎶€ MX-01P 钃濈墮浣庡姛鑰楋紙BLE 5.3锛夐€忎紶妯″潡鐨勫簳灞傞┍鍔紝鑱岃矗闄愬畾涓猴細

- **鐢熷懡鍛ㄦ湡**锛歊ES 纭浣?/ `AT+REBOOT` 杞浣?/ 鎭㈠鍑哄巶锛屼互鍙婃墜鍐屽己鍒剁殑
  `+READY` 鎻℃墜锛?- **鏁版嵁闈?*锛氶€忎紶鍙戦€侊紙`FFF1` 閫氱煡閫氶亾锛夈€佷腑鏂帴鏀?+ 鐜舰缂撳啿銆?  **AT 搴旂瓟涓庨€忎紶鏁版嵁鐨勪覆鍙ｅ垎娴?*锛?- **鎺у埗闈?*锛氭墜鍐岀 5~15 椤靛叏濂?AT 鎸囦护锛圡AC / 鍚嶇О / 骞挎挱 / 娉㈢壒鐜?/ 骞挎挱闂撮殧 /
  鐗堟湰 / 鍔熺巼 / 涓夋潯 UUID / 鑷畾涔夊箍鎾暟鎹?/ 璁惧鍒楄〃 / 鏂紑杩炴帴锛夛紱
- **鍔熻€?*锛欱RTS 鐫＄湢鑴氭帶鍒讹紝鍙戦€佷笌 AT 鍓嶈嚜鍔ㄥ敜閱掋€?
涓氬姟閫昏緫锛堣繛鎺ョ鐞嗙瓥鐣ャ€佹暟鎹崗璁皝瑁呫€佷綆鍔熻€楄皟搴︺€丱TA 绛夛級鏀惧湪 `App/` 灞傦紝
鏈ā鍧椾笉鎺哄拰銆?
**璧勬枡**锛歚C:\myProjects\PCB_Proj\docs\MX-01P瑙勬牸涔?V0.0.5.pdf`

| 鎵嬪唽椤?| 鍐呭 | 鏈┍鍔ㄧ殑瀵瑰簲瀹炵幇 |
|---|---|---|
| 绗?3 椤?| 鍑哄巶榛樿鍙傛暟 | 瑙佺 8 鑺傚疄娴嬭〃 |
| 绗?4 椤?| 寮曡剼瀹氫箟锛圕DS/LINK/FC/BRTS/RES锛?| 瑙?2.1 / 2.2 |
| 绗?5 椤?| 閫忎紶閫氶亾 FFF0 / FFF1 / FFF2 | 瑙?3.2 |
| 绗?6~15 椤?| AT 鎸囦护閫愭潯鏍煎紡涓庡簲绛?| `mx01p_read_*` / `mx01p_write_*` |

鍑哄巶榛樿锛?*115200-8-N-1**銆佸箍鎾紑銆佸箍鎾棿闅?200 ms銆佸姛鐜?0 dBm銆?鏈嶅姟/璇?鍐?UUID = `FFF0/FFF1/FFF2`銆佸悕绉?`NB-<MAC>`銆?
## 2. 纭欢璇存槑

### 2.1 鎺ュ彛涓庡紩鑴?
| 淇″彿 | MCU 寮曡剼 | 鏂瑰悜 | 璇存槑 |
|---|---|---|---|
| USART1_TX | PA9 (AF7) | MCU 鈫?妯″潡 | 鎺ユā鍧?RXD锛圥in13锛夛紝3.3 V TTL |
| USART1_RX | PA10 (AF7) | 妯″潡 鈫?MCU | 鎺ユā鍧?TXD锛圥in14锛夛紝3.3 V TTL |
| BT_RES | PC3 | 杈撳嚭 | 妯″潡 RES锛圥in16锛夛紝**浣庣數骞冲浣?* |
| BT_BRTS | PA1 | 杈撳嚭 | 妯″潡 BRTS锛圥in15锛夛紝**楂?鎮┖ = 鐫＄湢锛屼綆 = 宸ヤ綔** |

- USART1锛?15200-8-N-1锛屾棤纭欢娴佹帶锛屼腑鏂帴鏀讹紙`HAL_UARTEx_ReceiveToIdle_IT`锛?  涓?E22/GPS 鍚屼竴濂楀啓娉曪級锛孖RQ 浼樺厛绾?10銆?- `Core/Src/gpio.c` 涓婄數鎶?`BT_RES` 鍒濆鍖栦负**浣庣數骞?*锛堟ā鍧椾竴鐩村浜庡浣嶆€侊級锛?  蹇呴』鐢?`mx01p_init()` 閲婃斁锛涘悓鏃舵妸 `BT_BRTS` 鍒濆鍖栨垚**浣庣數骞?*锛堟ā鍧楀伐浣滄€侊級锛?  杩欎竴鐐瑰緢鍏抽敭锛欱RTS 涓€鏃︽偓绌烘垨鎷夐珮锛屾ā鍧椾細鐫¤繃鍘伙紝`+READY` 姘歌繙绛変笉鍒般€?- 妯″潡 VCC 鐢?`PW_EN` 鎺у埗鐨勭數婧愯建渚涚數锛坄main.c` 鍦ㄥ垵濮嬪寲鍓嶆媺楂橈級锛岄┍鍔ㄤ笉纰?PW_EN銆?
### 2.2 鏈澘娌℃湁鎺ョ殑涓夋牴绾匡紙鍐冲畾浜嗛┍鍔ㄧ殑鑳藉姏杈圭晫锛?
| 妯″潡鑴?| 鎵嬪唽璇箟 | 鏈澘 | 椹卞姩鍚庢灉 |
|---|---|---|---|
| CDS (IO22) | 楂?鎮┖ = AT 鍛戒护鏈夋晥锛岄潪 AT 鏁版嵁閫忎紶锛涗綆 = AT 鍏抽棴 | 鎮┖ = 楂?| **AT 鎸囦护濮嬬粓鏈夋晥**锛氫互 `AT+` 寮€澶寸殑杞借嵎浼氳妯″潡褰撳懡浠ゅ悆鎺夛紙瑙?10.2锛?|
| LINK (IO21) | 宸茶繛鎺ヨ緭鍑洪珮锛屾湭杩炴帴杈撳嚭浣?| 鎮┖ | 杩炴帴鐘舵€佸彧鑳介潬 `+CONNECTED`/`+DISCONN` 浜嬩欢涓?`AT+DEV?` 璺熻釜 |
| FC (IO20) | 楂?= 妯″潡涓嶅厑璁?MCU 缁х画鍙戜覆鍙ｆ暟鎹?| 鎮┖ | 娌℃湁纭欢鑳屽帇锛屽彂閫佽妭濂忕敱涓婂眰鎺у埗锛圔LE 杩炴帴闂撮殧 30 ms锛?|

### 2.3 澶嶄綅涓庝笂鐢垫椂搴?
| 鍦烘櫙 | 澶勭悊 | 鏁板€兼潵婧?|
|---|---|---|
| RES 浣庣數骞充繚鎸?| `MX01P_RESET_LOW_MS = 10 ms` | 鎵嬪唽鍙啓"鎷変綆澶嶄綅"锛屾棤涓嬮檺锛屽彇淇濆畧鍊?|
| 澶嶄綅閲婃斁 鈫?`+READY` | 椹卞姩杞绛夊緟锛岄粯璁や笂闄?3000 ms | 鎵嬪唽绗?6 椤靛娉?涓婄數鎴栭噸鍚垚鍔熲€︽敹鍒?+READY 鍚庢墠鑳芥墽琛屾寚浠ゅ拰鏁颁紶" |
| BRTS 鍞ら啋 鈫?鍙彂閫?| `MX01P_WAKE_MS = 10 ms` | 鎵嬪唽鏈粰锛屽疄娴?10 ms 绋冲畾锛堢 8 鑺傦級 |

鏈澘瀹炴祴锛氬浣嶉噴鏀惧埌 `+READY` 绾?**795~827 ms**锛圔LE 鍗忚鏍堝惎鍔ㄨ€楁椂锛夛紝
涓?`ready_ms` 瀛楁涓€鑷淬€?
## 3. 鍗忚瑕佺偣

### 3.1 AT 涓庡紓姝ヤ簨浠?
- 鍛戒护缁熶竴 `<CMD>\r\n`锛坄0x0D 0x0A` 缁撳熬锛夈€?- 璁剧疆绫诲洖 `OK\r\n` / `ERROR\r\n`锛涙煡璇㈢被鍥?`+KEY:VALUE\r\n`锛坄+NAME: xxx`銆?  `+UART: NUM` 杩欑被甯︾┖鏍肩殑锛岄┍鍔ㄧ粺涓€璺崇┖鏍煎悗鍙栧€硷級銆?- 涓夌寮傛浜嬩欢锛屽彲鑳藉湪浠绘剰鏃跺埢鎻掕繘鏉ワ細

| 浜嬩欢 | 鍚箟 | 椹卞姩鍔ㄤ綔 |
|---|---|---|
| `+READY` | 涓婄數/閲嶅惎瀹屾垚 | 缃?`ready_edge`锛屽洖璋?`MX01P_EVENT_READY` |
| `+CONNECTED:TYP,MAC` | 涓绘満杩炰笂 | 鏇存柊 `link_up/link_type/link_mac`锛屽洖璋?|
| `+DISCONN:TYP,MAC` | 杩炴帴鏂紑 | 鍚屼笂 |

`TYP = 1` 琛ㄧず杩炴帴鏂逛负涓绘満锛堟墜鏈猴級锛宍MAC` 涓哄绔?12 浣嶅崄鍏繘鍒跺湴鍧€銆?
### 3.2 鏁版嵁娴佸悜锛堢 5 椤?BLE 鍗忚璇存槑锛?
```
鎵嬫満 --鍐?FFF2--> 妯″潡 --涓插彛 TX--> MCU       锛圓PP 鍙戠粰 MCU锛?MCU --涓插彛 RX--> 妯″潡 --閫氱煡 FFF1--> 鎵嬫満      锛圡CU 鍙戠粰 APP锛?```

鍗筹細**MCU 寰€涓插彛鍐?= 鎵嬫満鏀跺埌閫氱煡锛涙墜鏈哄啓 FFF2 = MCU 浠庝覆鍙ｈ**銆?椹卞姩鍙仛閫忎紶锛屼笉瑙ｆ瀽涓氬姟鍐呭锛汳TU 鍒嗗寘鐢辨ā鍧楄嚜宸卞畬鎴愩€?
## 4. 鏈増鑼冨洿

| 缁村害 | v1锛堝綋鍓嶅疄鐜帮級 | 璇存槑 |
|---|---|---|
| 澶嶄綅 | RES 纭浣?+ `AT+REBOOT` 杞浣?+ `AT+RESET` 鎭㈠鍑哄巶 | 閮借绛?`+READY` |
| 鍙戦€?| 闃诲鍙戦€侊紝鍗曟 鈮?512 B | 瓒呰繃杩斿洖 `MX01P_ERR_PARAM`锛涜儗鍘嬮潬涓婂眰 |
| 鎺ユ敹 | IDLE 涓柇 + 1 KB 鐜舰缂撳啿 + 鍒嗗眰鍒嗘祦 | 婊℃椂涓㈡柊鏁版嵁锛屼笉瑕嗙洊鏈秷璐规暟鎹?|
| 鍙傛暟 | 鎵嬪唽鍏ㄥ AT锛歁AC/鍚嶇О/骞挎挱/娉㈢壒鐜?闂撮殧/鐗堟湰/鍔熺巼/涓夋潯 UUID/骞挎挱鏁版嵁 | 鍐欐尝鐗圭巼鍙帴鍙?115200锛堣 2.2銆?0.4锛?|
| 鐫＄湢 | BRTS 鎺у埗 + 鍙戦€?AT 鍓嶈嚜鍔ㄥ敜閱?| 鐫＄湢鏃舵ā鍧椾笉鑳藉彂涓插彛锛堟墜鍐?Pin15锛?|
| 鑷 | `mx01p_probe()`锛歩nit + 13 椤?AT + 鐫＄湢鑴?+ 鏁版嵁鍥炵幆 + 杞浣?| 缁撴灉鐣欏叏灞€鍙橀噺渚?J-Link 璇?|

**涓嶅湪鏈増**锛氬杩炴帴绠＄悊绛栫暐銆佽繛鎺ュ弬鏁板崗鍟嗭紙杩炴帴闂撮殧/MTU 鏃?AT 鎸囦护鍙皟锛夈€?鑷畾涔夊箍鎾暟鎹互澶栫殑骞挎挱鍖呭瓧娈点€丱TA 鍥轰欢鍗囩骇銆佹柇绾胯嚜鍔ㄩ噸杩炪€?
## 5. 杞欢鏋舵瀯

### 5.1 鏂囦欢鍒掑垎

| 鏂囦欢 | 鑱岃矗 |
|---|---|
| `mx01p.c/.h` | 椹卞姩鏍稿績锛氱敓鍛藉懆鏈熴€丷X 鍒嗘祦鍣ㄣ€佸悓姝?AT 寮曟搸銆佸弬鏁拌鍐欍€佺潯鐪犮€佽嚜妫€銆?*绾?C锛屼笉 include HAL/RTOS 澶存枃浠?* |
| `mx01p_port.c/.h` | 骞冲彴绉绘灞傦細USART1 涓柇鏀跺彂銆丷ES/BRTS 鎺у埗銆丠AL tick/delay銆佷复鐣屽尯銆?*鍞竴鍏佽鎺ヨЕ HAL 鐨勬枃浠?* |
| `CMakeLists.txt` | target `mx01p`锛屾敞鍐岃繘 `BSP_LIBS`锛坄BSP/CMakeLists.txt`锛?|

椹卞姩鏍稿績涓庣Щ妞嶅眰涔嬮棿鍙€氳繃 `mx01p_port.h` 鐨勪竴缁勫嚱鏁颁氦浜掞紝绛惧悕涓嶅惈 HAL 绫诲瀷锛?鎹?MCU/鎹覆鍙ｅ彧鏀?`mx01p_port.c`銆?
### 5.2 鏁版嵁娴?
```mermaid
flowchart LR
    A[MX-01P 妯″潡] -->|TXD 鈫?USART1_RX| B["HAL_UARTEx_RxEventCallback<br/>mx01p_port_rx_event"]
    B --> C["mx01p_rx_feed 鍒嗘祦鍣?br/>锛圛SR锛?]
    C -->|鎺у埗琛寍 D[搴旂瓟閭 / 浜嬩欢鍥炶皟]
    C -->|閫忎紶鏁版嵁| E["鐜舰缂撳啿 1 KB<br/>婊″垯涓㈡柊鏁版嵁"]
    E -->|mx01p_read| F[App 绾跨▼]
    F -->|mx01p_send| G["BRTS 鍞ら啋 鈫?HAL_UART_Transmit"]
    G --> A
    D --> H["mx01p_exchange<br/>鍚屾 AT 浜嬪姟"]
```

### 5.3 骞跺彂涓庝緷璧?
- 渚濊禆鏂瑰悜锛歚App` 鈫?`mx01p`锛坈ore锛夆啋 `mx01p_port` 鈫?HAL銆俢ore 涓?include 浠讳綍 HAL 澶存枃浠躲€?- **鎺ユ敹鏄?SPSC**锛歊X 涓柇鍙啓 `rx_head`锛屾秷璐圭嚎绋嬪彧鍐?`rx_tail`銆?- **鍒嗘祦鍣ㄤ笌璇荤紦鍐茬殑浜ゆ帴鐢ㄧ煭涓寸晫鍖?*锛歚mx01p_poll()` 鎶?鍋滃湪鍊欓€夎閲岀殑灏惧反"鍐茶繘
  鐜舰缂撳啿鏃朵細鍐?`rx_head`锛屽洜姝ょ敤 `mx01p_port_irq_lock()` 鍖呬綇锛堝彧鏈夊嚑鍗佹潯鎸囦护锛夈€?- **AT 浜嬪姟鏄悓姝ヤ笖鐙崰鐨?*锛歚at_pending` 鏈熼棿鍐嶆潵涓€鏉?AT 鐩存帴杩斿洖
  `MX01P_ERR_STATE`锛汚T 绫?API 涓庢暟鎹被 API **涓嶈兘**鍦ㄤ袱涓嚎绋嬮噷骞跺彂璋冪敤銆?  寤鸿鐢?App 鐨勪竴涓摑鐗欑嚎绋嬬嫭鍗犻┍鍔ㄣ€?- `mx01p_init()/mx01p_probe()` 鍦ㄤ富寰幆锛坄tx_kernel_enter()` 涔嬪墠锛夋墽琛岋紝鍐呴儴鍙敤
  `HAL_Delay` 涓庤疆璇紝涓嶄緷璧?RTOS锛汚T 绛夊緟寰幆鍚屾牱鏄?`HAL_Delay(1)` 杞
  锛堜笌 E22 涓€鑷达級锛屼笉瑕佸湪浼樺厛绾ч珮浜庤皟搴﹀櫒鍏抽敭绾跨▼鐨勫湴鏂归暱鏃堕棿璋冪敤銆?
## 6. API 濂戠害

### 6.1 鏁版嵁绫诲瀷

```c
typedef struct {
  uint8_t *rx_ring;          /* 璋冪敤鑰呭垎閰嶏紝瀹归噺蹇呴』鏄?2 鐨勫箓 */
  uint16_t rx_ring_size;     /* 寤鸿 1024 */
  mx01p_event_cb_t evt_cb;   /* 鍙€変簨浠跺洖璋冿紙READY / CONNECTED / DISCONN锛?*/
  void *user;
  uint32_t ready_timeout_ms; /* 0 = 榛樿 3000 ms */
} mx01p_cfg_t;

/* 浜嬩欢鍥炶皟锛欵VENT 绫诲瀷 + TYP + MAC锛圡AC 鎸囬拡浠呭湪鍥炶皟鏈熼棿鏈夋晥锛?*/
typedef void (*mx01p_event_cb_t)(mx01p_t *inst, mx01p_event_t event,
                                 uint8_t link_type, const char *mac);
```

杩斿洖鍊肩粺涓€涓?`mx01p_status_t`锛堣礋鍊煎嵆閿欒锛夛細`OK / ERR_PARAM / ERR_TIMEOUT /
ERR_IO / ERR_PROTOCOL / ERR_STATE / ERR_NOMEM`銆?
### 6.2 鎺ュ彛鍑芥暟

```c
/* 鐢熷懡鍛ㄦ湡 */
mx01p_status_t mx01p_init(mx01p_t *inst, const mx01p_cfg_t *cfg);
mx01p_status_t mx01p_reset(mx01p_t *inst);
mx01p_status_t mx01p_reboot(mx01p_t *inst);
mx01p_status_t mx01p_restore(mx01p_t *inst);

/* 鐫＄湢涓庨摼璺姸鎬?*/
mx01p_status_t mx01p_set_sleep(mx01p_t *inst, bool sleep);
bool  mx01p_is_asleep(const mx01p_t *inst);
bool  mx01p_is_connected(const mx01p_t *inst);
void  mx01p_get_link(const mx01p_t *inst, uint8_t *link_type, char *mac);

/* 鏁版嵁闈?*/
mx01p_status_t mx01p_send(mx01p_t *inst, const uint8_t *data, uint16_t len, uint32_t timeout_ms);
uint16_t mx01p_read(mx01p_t *inst, uint8_t *buf, uint16_t max_len);
uint16_t mx01p_available(const mx01p_t *inst);
void     mx01p_flush_rx(mx01p_t *inst);

/* 閫氱敤 AT */
mx01p_status_t mx01p_at_query(mx01p_t *inst, const char *cmd, const char *key,
                              char *out, uint16_t out_size, uint32_t timeout_ms);
mx01p_status_t mx01p_at_exec(mx01p_t *inst, const char *cmd, uint32_t timeout_ms);

/* 鍙傛暟蹇嵎鎺ュ彛锛堣秴鏃剁粺涓€ MX01P_AT_TIMEOUT_MS = 500 ms锛?*/
mx01p_status_t mx01p_read_mac/write_mac(...);       /* AT+MAC? / AT+MAC= */
mx01p_status_t mx01p_read_name/write_name(...);     /* AT+NAME? / AT+NAME= */
mx01p_status_t mx01p_read_adv/set_adv(...);         /* AT+ADV? / AT+ADV= */
mx01p_status_t mx01p_read_baud/write_baud(...);     /* AT+UART? / AT+UART= */
mx01p_status_t mx01p_read/write_adv_interval(...);  /* AT+AINTVL */
mx01p_status_t mx01p_read_version(...);             /* AT+VER? */
mx01p_status_t mx01p_read/write_tx_power(...);      /* AT+TXPOWER */
mx01p_status_t mx01p_read/write_uuid(...);          /* AT+UUIDS/UUIDN/UUIDW */
mx01p_status_t mx01p_read/write_adv_data(...);      /* AT+AMDATA */
mx01p_status_t mx01p_disconnect(mx01p_t*, uint8_t mode); /* AT+DISCONN= */
mx01p_status_t mx01p_read_link(mx01p_t*, uint8_t *typ, char *mac); /* AT+DEV? */

/* 涓柇/杞 + 鑷 */
void mx01p_rx_feed(mx01p_t *inst, const uint8_t *data, uint16_t len);
void mx01p_poll(mx01p_t *inst);
bool mx01p_probe(mx01p_t *inst, const mx01p_cfg_t *cfg, mx01p_probe_t *out);
```

### 6.3 浣跨敤绀轰緥

```c
static uint8_t g_bt_rx_ring[1024];
static mx01p_t g_bt;

static void bt_evt(mx01p_t *inst, mx01p_event_t ev, uint8_t typ, const char *mac)
{
  /* ISR 涓婁笅鏂囷細鍙疆鏍囧織 */
  if (ev == MX01P_EVENT_CONNECTED) {
    g_bt_connected = true;
  }
}

void app_bt_init(void)
{
  mx01p_cfg_t cfg = {
    .rx_ring = g_bt_rx_ring,
    .rx_ring_size = sizeof(g_bt_rx_ring),
    .evt_cb = bt_evt,
  };

  if (mx01p_init(&g_bt, &cfg) != MX01P_OK) {
    /* 鏌?VCC / RES / USART1=115200 */
  }
  (void)mx01p_write_name(&g_bt, "HardBrick");   /* 绔嬪嵆鐢熸晥 */
}

void app_bt_thread_entry(void)
{
  uint8_t buf[64];

  for (;;) {
    uint16_t n = mx01p_read(&g_bt, buf, sizeof(buf));
    if (n != 0u) {
      /* 鎵嬫満鍐?FFF2 鐨勬暟鎹?*/
    }
    if (/* 鏈夊緟鍙戞暟鎹?*/) {
      (void)mx01p_send(&g_bt, tx, tx_len, 100u);   /* 璧?FFF1 閫氱煡缁欐墜鏈?*/
    }
    tx_thread_sleep(1);   /* 10 ms */
  }
}
```

## 7. RX 鍒嗘祦瑙勫垯锛堟湰椹卞姩鐨勬牳蹇冭璁＄偣锛?
鏈澘 CDS 鎮┖锛屾墍浠?**AT 搴旂瓟涓庢墜鏈洪€忎紶鏁版嵁鍦ㄥ悓涓€鏍?RX 绾夸笂浜ら敊鍑虹幇**锛?椹卞姩鎸夎鍋氬垎娴侊紙`mx01p_rx_feed` / `mx01p_line_complete`锛夛細

| 鏀跺埌鐨勫唴瀹?| 鍒ゅ畾 | 鍘诲悜 |
|---|---|---|
| 琛岄涓嶆槸 `+` / `O` / `E` | 涓€瀹氫笉鏄帶鍒惰 | **閫愬瓧鑺傜洿鎺ヨ繘鏁版嵁缂撳啿**锛堥浂寤惰繜锛?|
| `+READY` | 寮傛浜嬩欢 | `ready_edge` + 鍥炶皟 |
| `+CONNECTED:鈥 / `+DISCONN:鈥 | 寮傛浜嬩欢 | 鏇存柊閾捐矾鐘舵€?+ 鍥炶皟 |
| 鍏跺畠 `+鈥 琛岋紝涓?*鏈?AT 浜嬪姟鍦ㄧ瓑** | AT 搴旂瓟 | 搴旂瓟閭锛堝幓 CRLF锛?|
| 鍏跺畠 `+鈥 琛岋紝**绌洪棽鏃?* | 浠?`+` 寮€澶寸殑浜岃繘鍒惰浇鑽?| 閫€鍥炴暟鎹紦鍐?|
| `OK` / `ERROR`锛屼笖鏈?AT 浜嬪姟鍦ㄧ瓑 | AT 搴旂瓟 | 搴旂瓟閭 |
| 鍊欓€夎瓒呰繃 `MX01P_RX_IDLE_FLUSH_MS`(5 ms) 娌″姩闈?| 涓嶅彲鑳芥槸瀹屾暣鎺у埗琛?| 鏁翠綋閫€鍥炴暟鎹紦鍐?|
| 鍊欓€夎瓒呰繃 95 瀛楄妭 | 鎵嬪唽鏈€闀垮簲绛斾篃瑁呭緱涓?| 鏁翠綋閫€鍥炴暟鎹紦鍐插苟鏀逛负鐩撮€?|

鍥犳锛氭櫘閫氫簩杩涘埗杞借嵎闆堕澶栧欢杩燂紱鍙湁**浠?`+`/`O`/`E` 寮€澶翠笖涓嶅惈 `\n`** 鐨勮浇鑽?鏈€澶氭櫄 5 ms 鍒拌揪銆傝繖涓唬浠锋槸涓轰簡涓嶅悶鎺夊紓姝ヤ簨浠讹紝宸插湪绗?8 鑺傜敤鍥炵幆瀹炴祴楠岃瘉銆?
## 8. 涓婄數鑷涓庡疄娴嬬粨鏋?
`main.c` 鍦?`tx_kernel_enter()` 涔嬪墠璋冪敤 `bt_selftest()` 鈫?`mx01p_probe()`銆?缁撴灉鍚屾椂鍐欒繘**鍏ㄥ眬** `g_bt_probe`锛堝埢鎰忎笉鍔?`static`锛夛紝J-Link 鐩存帴鎸夌鍙疯锛?
```
stage = 5        init_st = 0       ready_ms = 795~827   reset_retry = 0
mac = "C329B0CFB3DB"              name = "NB-C329B0CFB3DB"
version = "V0.0.8"                baud_code = 5 (115200)
adv_on = 1        interval_ms = 200      power_code = 5 (0 dBm)
uuid = FFF0 / FFF1 / FFF2
link_st = -4锛堟棤鎵嬫満鍦ㄦ梺锛屽睘棰勬湡锛屼笉璁″け璐ワ級
sleep_st = 0      sleep_blocked = 1  鈫?BRTS 鐫＄湢鑴氭湁鏁?loop_st = 0       loopback = "+VER:V0.0.8\r\n"  鈫?鏁版嵁閫氳矾鍥炵幆
reboot_st = 0     鈫?杞浣嶅悗閲嶆柊绛夊埌 +READY
at_total = 14     at_fail = 0   rx_dropped = 0   resp_overrun = 0
```

RTT 杈撳嚭褰㈠锛?
```
[BT] probe=1 stage=5 init=0 ready=805 ms retry=0
[BT] mac=C329B0CFB3DB(0) name=NB-C329B0CFB3DB(0)
[BT] ver=V0.0.8(0)
[BT] uart=115200 code=5(0) adv=1(0) aintvl=200 ms(0)
[BT] txpower_code=5(0)
[BT] uuid s=FFF0 n=FFF1 w=FFF2 (0)
[BT] sleep st=0 blocked=1 wake=0
[BT] loop_st=0 len=13 text=+VER:V0.0.8
[BT] reboot_st=0
[BT] link_st=-4 connected=0 dev_mac=-
[BT] at=14 fail=0 drop=0 overrun=0
```

瀹炴祴鏂规硶锛歚mcu_flash` 鐑у綍鍚庣瓑 8 s锛岀敤 J-Link 璇?`g_bt_probe`锛?杩炵画涓ゆ纭浣嶇粨鏋滀竴鑷达紙`ready_ms` 805 / 827 ms锛夈€?
楠屾敹娓呭崟锛?
- [x] `mx01p_init()` 鑳介噴鏀?RES 骞剁瓑鍒?`+READY`锛宍ready_ms` 绋冲畾鍦?0.8 s 宸﹀彸锛?- [x] 13 鏉?AT 鏌ヨ鍏ㄩ儴鎴愬姛锛歁AC / 鍚嶇О / 鐗堟湰 / 娉㈢壒鐜?/ 骞挎挱 / 闂撮殧 / 鍔熺巼 / 涓夋潯 UUID / 璁惧鍒楄〃锛?- [x] `AT+UART?` 杩斿洖鐮佸€?5锛?15200锛夛紝涓?CubeMX 鐨?USART1 涓€鑷达紱
- [x] UUID 涓哄嚭鍘?`FFF0/FFF1/FFF2`锛屽悕绉扮鍚堟墜鍐岀殑 `NB-<MAC>` 榛樿瑙勫垯锛?- [x] BRTS 鎷夐珮鍚?`AT+MAC?` 鏃犲簲绛斻€佹媺浣庡悗鎭㈠ 鈫?鐫＄湢鑴氭柟鍚戞纭紱
- [x] 浠庢暟鎹€氳矾鍙?`AT+VER?` 鑳芥妸 `+VER:V0.0.8\r\n` 褰撹浇鑽锋敹鍥炴潵 鈫?鍙戦€併€佸垎娴併€佹帴鏀朵笁娈甸兘閫氾紱
- [x] `AT+REBOOT=1` 鍚庤兘閲嶆柊绛夊埌 `+READY` 骞剁户缁煡璇紱
- [x] 鍏ㄧ▼ `rx_dropped = 0`銆乣resp_overrun = 0`锛堝垎娴佹病鏈変涪瀛楄妭銆佹病鏈夋姠閭锛夛紱
- [ ] **涓庢墜鏈哄疄娴嬮€忎紶**锛欰PP 鍐?FFF2 鈫?`mx01p_read()` 鏀跺埌锛沗mx01p_send()` 鈫?APP 鏀跺埌閫氱煡
      锛堥渶瑕佹墜鏈哄湪鏃侊紝瑙?10.5锛夛紱
- [ ] 闀挎椂闂磋繍琛岋紙>1 h锛塦rx_dropped` 涓嶅闀裤€?
## 9. 閿欒澶勭悊

| 鍦烘櫙 | 澶勭悊 |
|---|---|
| 鐜舰缂撳啿婊?| 涓㈠純鏂板埌鐨勬暣娈甸噷鏀句笉涓嬬殑閮ㄥ垎锛宍rx_dropped` 绱姞锛涗笉瑕嗙洊鏈秷璐规暟鎹?|
| ORE / 甯ч敊璇?| `HAL_UART_ErrorCallback` 鈫?`mx01p_port_uart_error()` 澶嶄綅鎺ユ敹骞?rearm锛宍rx_loss` 缃綅 |
| AT 鏃犲簲绛?| 杩斿洖 `MX01P_ERR_TIMEOUT`锛宍at_pending` 涓€瀹氳娓呮帀锛堜笉浼氬崱姝诲悗缁懡浠わ級 |
| 妯″潡鍥?`ERROR` | 杩斿洖 `MX01P_ERR_PROTOCOL` |
| 搴旂瓟鏍煎紡涓嶇锛堢己 key / 鍊间笉鏄暟瀛楋級 | `MX01P_ERR_PROTOCOL` |
| 绛?`+READY` 瓒呮椂 | `init/reset` 杩斿洖 `MX01P_ERR_TIMEOUT`锛宍ready` 淇濇寔 false锛屾墍鏈?AT 杩斿洖 `MX01P_ERR_STATE`锛涘彲鍙嶅 init 閲嶈瘯锛堣嚜妫€閲屽氨閲嶈瘯涓€娆★級 |
| 搴旂瓟閭琚鐩?| `resp_overrun` 绱姞锛堣鏄庢湁浜哄湪骞跺彂鐢ㄩ┍鍔ㄧ▼搴忥級 |
| 鐫＄湢涓皟鐢?AT 绫?API | 鑷姩鍞ら啋鍚庡啀鍙戯紙`mx01p_send` 鍚屾牱锛夛紝涓嶄細鍋囪鎴愬姛 |

## 10. 宸茬煡闄愬埗涓庡悗缁?
1. **AT 涓庨€忎紶鏁版嵁涓嶈兘骞跺彂**锛欰T 浜嬪姟鏈熼棿鏀跺埌鐨勩€佷互 `+` 寮€澶寸殑杞借嵎浼氳褰撴垚鍊欓€夊簲绛旓紝
   鏈€鍧忚涓㈠純銆侫pp 搴旇"瑕佷箞鍦ㄩ厤缃紝瑕佷箞鍦ㄤ紶鏁版嵁"锛屼笉瑕佷袱杈瑰悓鏃舵潵銆?2. **CDS 鎮┖瀵艰嚧 `AT+` 杞借嵎鏈夐闄?*锛氭ā鍧椾細鎶婅棣栦负 `AT+` 鐨勬暟鎹綋鍛戒护瑙ｆ瀽骞跺洖
   `ERROR`锛岄┍鍔ㄦ棤娉曢樆姝€備笟鍔″崗璁簲閬垮厤璁╄浇鑽蜂互 `AT+` 寮€澶达紝鎴栧悗缁妸 CDS 鎺ュ埌
   GPIO 涓婏紝鍦ㄤ紶鏁版嵁鏃舵媺浣庡畠锛堟湰椹卞姩棰勭暀浜嗚繖绉嶆墿灞曠殑绌洪棿锛屼絾鏈澘娌″竷绾匡級銆?3. **娌℃湁纭欢娴佹帶**锛欶C 鑴氭湭鎺ワ紝妯″潡鏃犳硶鍙仠 MCU銆備竴娆?`mx01p_send()` 鍙 鈮?12 B
   灏辩敱 HAL 闃诲鍙戝畬锛屾暣鍖呮槸鍚︾湡琚?BLE 鍙戝嚭鍘婚┍鍔ㄤ笉鐭ラ亾锛涙湭杩炴帴鏃舵ā鍧椾細闈欓粯涓㈠純銆?4. **娉㈢壒鐜囩炕杞?*锛歎SART1 琚?CubeMX 鍥哄畾鎴?115200銆傚鏋滄ā鍧楅噷瀛樼殑娉㈢壒鐜囪鏀规垚鍒殑
   鍊硷紙鎹㈣繃鍥轰欢/鍒汉鐢ㄨ繃锛夛紝`+READY` 浼氬彉鎴愪贡鐮?鈫?`init` 瓒呮椂銆?   鎭㈠鍔炴硶锛氭妸 host 娉㈢壒鐜囦复鏃舵敼鎴愭ā鍧楅噷鐨勫€硷紝鎵ц `AT+UART=5`锛涙垨鎸夋墜鍐?`AT+RESET=1`
   鎭㈠鍑哄巶锛堟敞鎰忥細**MAC 鍦板潃鏀瑰姩涓嶅彲鎭㈠**锛夈€俙mx01p_write_baud()` 鍥犳鍙帴鍙?   `MX01P_BAUD_115200`銆?5. **閫忎紶瑕佺敤鎵嬫満楠岃瘉**锛氱 8 鑺傜殑鍥炵幆鍙兘璇佹槑"MCU鈫旀ā鍧?杩欎竴娈碉紱
   "妯″潡鈫旀墜鏈?蹇呴』鐢?nRF Connect 涔嬬被鐨?APP 瀹炴祴锛屾湰椹卞姩涓嶆彁渚涜鑳藉姏銆?6. **鍞ら啋寤舵椂鏄粡楠屽€?*锛氭墜鍐屾病缁?BRTS 鍞ら啋鏃堕棿锛宍MX01P_WAKE_MS = 10 ms` 鏄疄娴嬬ǔ瀹氬€硷紱
   濡傛灉鎹㈠浐浠跺悗鍋跺彂涓㈤瀛楄妭锛屽厛鎶婅繖閲屽姞澶у埌 20~50 ms 鍐嶆煡鍒銆?7. **`AT+DEV?` 鍙彇绗竴鏉?*锛氭ā鍧楁敮鎸佸杩炴帴鏃朵細鍥炲琛?`+DEV:`锛宍mx01p_read_link()`
   鍙彇绗竴琛岋紱瑕佽窡韪叏閮ㄨ繛鎺ヨ鐢ㄤ簨浠跺洖璋冦€?8. **AT 绛夊緟鏄繖绛?*锛歚mx01p_exchange()` 鐢?`HAL_Delay(1)` 杞搴旂瓟锛屼竴娆′簨鍔℃渶澶?   500 ms銆傛斁鍦ㄧ嫭绔嬬嚎绋嬮噷璋冪敤锛屼笉瑕佸湪鎺у埗鐜垨楂樹紭鍏堢骇绾跨▼閲岄暱鏃堕棿璺戙€?