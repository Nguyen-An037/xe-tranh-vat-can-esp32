# -*- coding: utf-8 -*-
# ============================================================================
# XE TỰ HÀNH TRÁNH VẬT CẢN - THUẬT TOÁN MÁY TRẠNG THÁI (FSM)
# ============================================================================
# Nền tảng : ESP32 DevKit V1 + MicroPython
# Cơ cấu   : 01 L298N (2 kênh điều khiển 4 motor), HC-SR04, servo SG90
# Phiên bản: FSM gọn - cơ chế thoát hiểm quay 0,5 giây (08/11/2026)
#
# MỤC TIÊU ĐIỀU KHIỂN
#   1. Chạy thẳng theo hai mức tốc độ, dừng khi gặp vật cản.
#   2. Quét hai bên; ưu tiên hướng đo được khoảng cách lớn hơn.
#   3. Nếu rẽ thất bại, trở về hướng cũ và thử hướng còn lại.
#   4. Nếu hai bên bị chắn (ví dụ hốc chữ U), lùi từng nhịp và quét lại.
#   5. Sau ba nhịp lùi mà chưa có lối, có thể THỬ quay thoát hiểm 0,5 s.
#   6. Giới hạn số lần xử lý; mất dữ liệu cảm biến thì dừng an toàn.
#
# 5 TRẠNG THÁI FSM
#   DRIVE : đọc khoảng cách trước; chạy nhanh/chậm hoặc dừng.
#   SCAN  : quét hai phía và sắp xếp hướng ứng viên.
#   TURN  : quay hướng ứng viên, xác nhận khoảng cách trước.
#   BACK  : lùi một nhịp, sau đó quay lại SCAN ở vị trí mới.
#   SAFE  : dừng khi mất tín hiệu hoặc hết giới hạn thoát hiểm.
#
# QUY ƯỚC CHIỀU ĐỘNG CƠ (ĐÃ ĐƯỢC KIỂM THỬ VỚI XE)
#   Kênh A - PHẢI: IN1=22, IN2=21, ENA=23.
#   Kênh B - TRÁI: IN3=19, IN4=18, ENB=5.
#   move(right=1, left=1)  : tiến thẳng.
#   move(right=-1, left=-1): lùi thẳng.
#   move(right=1, left=-1) : rẽ trái tại chỗ.
#   move(right=-1,left=1)  : rẽ phải tại chỗ.
#
# PHẠM VI CHỈNH SỬA
#   Bản này CHỈ sắp xếp lại nhãn đề và BỔ SUNG CHÚ THÍCH. Tất cả lệnh,
#   giá trị, điều kiện, thứ tự xử lý và cấu trúc FSM đều GIỮ NGUYÊN
#   so với file 'xe_tranh_vat_can_FSM_thoat_hiem_05s_gon.py'.
#
# GIỚI HẠN CẦN GHI RÕ KHI BÁO CÁO
#   - Không có encoder/góc quay: 0,4 s và 0,5 s chỉ là giá trị THỰC NGHIỆM,
#     không bảo đảm xe quay đúng một góc tuyệt đối.
#   - Không có cảm biến sau: lùi tự động chỉ nên thử khi phía sau đã trống.
#   - Mất ECHO không được đồng nhất với 'khong co vat can'.
#   - Không sử dụng OLED/web trong file này; nếu có, mô tả ở module khác.
# ============================================================================

from machine import Pin, PWM, time_pulse_us
import time

# ============================================================================
# 1. KHAI BÁO PHẦN CỨNG VÀ SƠ ĐỒ CHÂN
# ============================================================================
# GPIO của hai kênh L298N đã được chốt trên xe thật, KHÔNG đảo hai bên.
# Hai motor cùng phía được nối song song vào một kênh của L298N.
IN1 = Pin(22, Pin.OUT, value=0)  # PHAI
IN2 = Pin(21, Pin.OUT, value=0)
ENA = PWM(Pin(23), freq=1000, duty_u16=0)
IN3 = Pin(19, Pin.OUT, value=0)  # TRAI
IN4 = Pin(18, Pin.OUT, value=0)
ENB = PWM(Pin(5), freq=1000, duty_u16=0)
TRIG = Pin(4, Pin.OUT, value=0)
ECHO = Pin(15, Pin.IN)
servo = PWM(Pin(13), freq=50)

# ============================================================================
# 2. THAM SỐ KỸ THUẬT / NGƯỠNG RA QUYẾT ĐỊNH
# ============================================================================
# PWM 16-bit: 0 = tắt, 65535 = duty lớn nhất.
# Muốn thay đổi thông số, sửa tại đây và KIỂM THỬ LẠI trên xe thật.
# Tránh sửa nhiều ngưỡng cùng lúc vì khó xác định nguyên nhân lỗi.
DRIVE_SPEED = 35000
SLOW_SPEED = 27000
BACK_SPEED = 48000
TURN_SPEED = 54000
# Đơn vị thời gian là giây; điện áp pin ảnh hưởng đến vận tốc thực tế.
BACK_TIME = 0.4               # Do lai quang duong lui THUC TE tai PWM 48000
TURN_TIME = 0.4               # Quay thong thuong
ESCAPE_TURN_TIME = 0.5        # Chi quay thoat hiem sau khi lui/quet 3 lan

# Điều kiện trên trục TRƯỚC; đơn vị cm.
# Khoảng STOP_DISTANCE < d <= SLOW_DISTANCE: xe tiến chậm.
# d <= STOP_DISTANCE: xe dừng để xử lý vật cản.
STOP_DISTANCE = 32
TOO_CLOSE_DIST = 18
SLOW_DISTANCE = 75
# Ngưỡng độ rộng cho phép HƯỚNG RẼ thông thường.
# Phía bên < SIDE_CLEAR được đánh giá là bị chắn, tuy nhiên đây chỉ là
# ước lượng bằng một tia sóng siêu âm; không phải bản đồ không gian.
SIDE_CLEAR = 38
ESCAPE_MIN_SIDE = 24         # Nguong cho phep THU quay, khong phai duong chac chan thoang
RESUME_DISTANCE = 40          # Phai lon hon STOP_DISTANCE

# Góc servo theo quy ước cơ khí của xe (không tự ý đảo trái/phải).
SERVO_RIGHT = 0
SERVO_CENTER = 90
SERVO_LEFT = 180
SERVO_WAIT_MS = 650
PING_INTERVAL_MS = 65

# Giới hạn xử lý trong MỘT ĐỢT né vật cản, không phải mỗi vòng main.
# Sau khi xe tiến ổn định CLEAR_RUN_MS mới xóa bộ đếm.
MAX_BACK_STEPS = 3
MAX_SCANS = 6
CLEAR_RUN_MS = 1500           # Da tien lien tuc du lau moi reset bo dem
# CẢNH BÁO: True cho phép lùi khi KHÔNG có cảm biến phía sau.
# Nếu thử trong khu vực không kiểm soát, đổi False trước khi cấp nguồn.
ALLOW_BLIND_BACK = True       # Sua False neu KHONG kiem soat duoc phia sau

# ============================================================================
# 3. CƠ CẤU CHẤP HÀNH: SERVO VÀ ĐỘNG CƠ DC
# ============================================================================
# Các hàm bao giúp step_* mô tả hành động bằng tên rõ ràng.
# move()/set_servo_angle() là các hàm nền, giữ nguyên từ bản đã chạy.
# Servo SG90: góc [0,180] chuyển thành duty_u16 trên PWM 50 Hz.
# 1638 -> ~500 µs; 7864 -> ~2400 µs (giá trị đang dùng trên xe).
# Mỗi lệnh đặt góc chờ 650 ms để servo di chuyển đến vị trí.
def set_servo_angle(angle):
    angle = max(0, min(180, angle))
    duty = int(1638 + (7864 - 1638) * angle / 180)
    servo.duty_u16(duty)
    time.sleep_ms(SERVO_WAIT_MS)


# Ngắt PWM VÀ reset bốn chân chiều để motor không tiếp tục quay.
def stop():
    ENA.duty_u16(0)
    ENB.duty_u16(0)
    IN1.off()
    IN2.off()
    IN3.off()
    IN4.off()


# right/left: 1=tiến; -1=lùi; 0=dừng một bên (nếu cần).
# Hàm có 60 ms thời gian nghỉ MỖI LẦN GỌI, kể cả không đổi chiều.
# Chú ý: gọi lặp lại hàm này khi đang tiến sẽ làm xe bị giật; vì vậy
# change_forward_speed() chỉ thay duty_u16 nếu đang ở chế độ RUN/SLOW.
def move(right, left, speed=DRIVE_SPEED):
    stop()
    time.sleep_ms(60)
    IN1.value(1 if right == 1 else 0)
    IN2.value(1 if right == -1 else 0)
    IN3.value(1 if left == 1 else 0)
    IN4.value(1 if left == -1 else 0)
    ENA.duty_u16(speed if right else 0)
    ENB.duty_u16(speed if left else 0)


def forward():
    move(1, 1, DRIVE_SPEED)


def forward_slow():
    move(1, 1, SLOW_SPEED)


def backward():
    move(-1, -1, BACK_SPEED)


def turn_right():
    move(-1, 1, TURN_SPEED)


def turn_left():
    move(1, -1, TURN_SPEED)


# Quay tại chỗ theo hướng đã chọn và tự dừng sau duration giây.
# Không có IMU/encoder nên góc quay phụ thuộc pin, ma sát và thời gian.
def rotate(direction, duration=TURN_TIME):
    if direction == 'LEFT':
        turn_left()
    else:
        turn_right()
    time.sleep(duration)
    stop()


# Dùng khi quay một hướng thất bại: quay theo chiều ngược lại
# với thời gian TƯƠNG ỨNG để xấp xỉ về hướng ban đầu.
def opposite(direction):
    return 'RIGHT' if direction == 'LEFT' else 'LEFT'


# ============================================================================
# 4. THU THẬP SỐ LIỆU HC-SR04 VÀ QUÉT KHÔNG GIAN
# ============================================================================
# HC-SR04 được kích bằng xung TRIG 10 µs; đo độ rộng xung ECHO.
# d(cm) = thời_gian_µs × 0,0343 / 2.
# Thời gian giữa hai xung trigger >= 65 ms để hạn chế nhiễu phép đo.
last_trigger_ms = None


def single_distance():
    """Tra ve so cm hop le hoac None neu mat ECHO/loi.
    None KHONG dong nghia duong trong; khong doi thanh 350 cm.
    """
    global last_trigger_ms
    if last_trigger_ms is not None:
        elapsed = time.ticks_diff(time.ticks_ms(), last_trigger_ms)
        if elapsed < PING_INTERVAL_MS:
            time.sleep_ms(PING_INTERVAL_MS - elapsed)
    TRIG.off()
    time.sleep_us(4)
    last_trigger_ms = time.ticks_ms()
    TRIG.on()
    time.sleep_us(10)
    TRIG.off()
    # Timeout 25 ms: nếu không có ECHO, không mặc định là đường trống.
    try:
        pulse = time_pulse_us(ECHO, 1, 25000)
    except OSError:
        return None
    if pulse <= 0:
        return None
    d = pulse * 0.0343 / 2
    if d > 400:
        return None
    return round(max(d, 2.0), 1)


# Đo nhiều lần, trả về số đo GẦN NHẤT (nếu có).
# Phương pháp này ưu tiên an toàn trước vật cản gần; song một kết quả
# nhiễu đo được rất gần có thể khiến xe dừng sớm. Báo cáo cần nêu rõ.
def measure(samples=2):
    """Lay so do GAN NHAT trong nhung lan co ECHO hop le.
    Chi can 1 lan hop le; khong doi 2/3 lan deu hop le nhu code cu.
    """
    values = []
    for _ in range(samples):
        d = single_distance()
        if d is not None:
            values.append(d)
    return min(values) if values else None


# Quét tại một vị trí: PHẢI(0) -> GIỮA(90) -> TRÁI(180) -> GIỮA(90).
# Sau mỗi lần quay servo chờ đủ thời gian, mỗi bên đo hai lần.
# Không tự lặp scan_sides trong hàm: FSM chỉ gọi khi state == SCAN.
def scan_sides():
    """Quet PHAI -> GIUA (250ms) -> TRAI -> GIUA mot lan/vi tri."""
    stop()
    set_servo_angle(SERVO_RIGHT)
    right = measure(2)
    # Di qua GIUA khi servo quet tu PHAI sang TRAI, giong code da test.
    servo.duty_u16(int(1638 + (7864 - 1638) * 90 / 180))
    time.sleep_ms(250)
    set_servo_angle(SERVO_LEFT)
    left = measure(2)
    set_servo_angle(SERVO_CENTER)
    print('SCAN | LEFT:', left, '| RIGHT:', right)
    return left, right


# Đầu ra là DANH SÁCH hướng ưu tiên: ['LEFT', 'RIGHT'] hoặc ít hơn.
# - Số đo >= SIDE_CLEAR: được đưa vào danh sách, sắp xếp giảm dần.
# - Số đo < SIDE_CLEAR: loại bỏ vì phía đó chưa đủ rộng.
# - None (mất ECHO): có thể đưa vào danh sách ĐỂ THỬ QUAY, nhưng phải
#   đo lại TRƯỚC MẶT xe và xác nhận > RESUME_DISTANCE mới được tiến.
# Đây không phải thuật toán nhận dạng hình học chữ U; chỉ là quy tắc
# dựa trên khoảng cách trục trước và hai hướng bên.
def turn_options(left, right):
    """Uu tien huong DA DO du rong; sau do moi thu huong mat ECHO.
    Huong do duoc < SIDE_CLEAR la BI CHAN, khong dua vao danh sach.
    Huong None chi duoc phep QUAY THU va XAC NHAN, khong tu dong tien.
    """
    measured = []
    unknown = []
    for direction, d in (('LEFT', left), ('RIGHT', right)):
        if d is None:
            unknown.append(direction)
        elif d >= SIDE_CLEAR:
            measured.append((direction, d))
    measured.sort(key=lambda pair: pair[1], reverse=True)
    return [direction for direction, _ in measured] + unknown


# ============================================================================
# 5. MÁY TRẠNG THÁI HỮU HẠN (FSM) - VÒNG LẶP CHÍNH DUY NHẤT
# ============================================================================
# BẢNG CHUYỂN TRẠNG THÁI (tóm tắt):
#   DRIVE -> SCAN  : vật cản <= 32 cm.
#   DRIVE -> BACK  : vật cản quá sát <= 18 cm.
#   DRIVE -> SAFE  : mất ECHO phía trước sau khi đo lại.
#   SCAN  -> TURN  : có hướng ứng viên, bao gồm hướng không có ECHO.
#   SCAN  -> BACK  : không có hướng, chưa hết giới hạn lùi.
#   TURN  -> DRIVE : sau quay, đo trước > 40 cm.
#   TURN  -> TURN  : hướng thứ nhất thất bại, thử hướng thứ hai.
#   TURN  -> BACK  : tất cả hướng đều thất bại, ECHO GIỮA còn hợp lệ.
#   BACK  -> SCAN  : lùi 0,4 s, đo tại vị trí mới.
#   *     -> SAFE  : hết giới hạn hoặc cảm biến không hợp lệ.
#   SAFE  -> *     : chỉ tự phục hồi nếu safe_kind='SENSOR' va co ECHO.
#
# Chú ý: SAFE với kind='LIMIT' KHÔNG tự chạy lại, cần can thiệp thủ công.
# Chỉ có một biến trạng thái được xử lý theo thứ tự trong main().
state = 'DRIVE'            # DRIVE / SCAN / TURN / BACK / SAFE
mode = 'STOP'              # STOP / RUN / SLOW
safe_reason = ''
safe_kind = ''              # SENSOR (tu hoi phuc) / LIMIT (can reset)
sensor_good = 0
back_steps = 0
scan_count = 0
choices = []
choice_idx = 0
# recovery_pending: vừa né vật cản thành công; CẦN tiến ổn định 1,5 s
# mới xóa back_steps/scan_count, giảm lặp tiến–lùi liên tục.
recovery_pending = False
clear_since_ms = None
escape_turn = False           # Chi bat khi thu quay thoat hiem
escape_used = False           # Khong thu lai 0.5s trong cung chu ky ne


# Chuyển về dừng an toàn và in lý do để truy vết trên Thonny.
# SENSOR: không đo được -> thử phục hồi; LIMIT: hết giới hạn -> dừng.
def enter_safe(reason, kind='LIMIT'):
    global state, mode, safe_reason, safe_kind, sensor_good
    stop()
    mode = 'STOP'
    state = 'SAFE'
    safe_reason = reason
    safe_kind = kind
    sensor_good = 0
    print('SAFE_STOP |', reason, '| kind:', kind,
          '| back:', back_steps, '| scans:', scan_count)


# Tránh giật khi chuyển RUN <-> SLOW: nếu xe ĐANG TIẾN, chỉ sửa PWM.
# Nếu xe đã STOP, mới gọi forward()/forward_slow() để khởi động.
def change_forward_speed(want):
    """Neu xe dang tien, chi sua PWM de tranh giat vi move() nghi 60ms."""
    global mode
    if want == mode:
        return
    if mode == 'STOP':
        if want == 'RUN':
            forward()
        else:
            forward_slow()
    else:
        speed = DRIVE_SPEED if want == 'RUN' else SLOW_SPEED
        ENA.duty_u16(speed)
        ENB.duty_u16(speed)
    mode = want
    print('DRIVE |', want)


# -------------------------------- 5.1 DRIVE --------------------------------
# Có ba nhánh: mất ECHO -> kiểm tra; gần vật cản -> dừng; còn lại -> tiến.
# Mỗi chu kỳ main đo một lần (không đo trung vị liên tục).
def step_drive():
    global state, mode, clear_since_ms, recovery_pending
    global back_steps, scan_count, escape_used
    d = single_distance()
    if d is None:
        if mode == 'RUN':
            change_forward_speed('SLOW')
        # Kiểm tra bổ sung để loại một lần mất ECHO ngẫu nhiên.
        d = measure(2)
        if d is None:
            enter_safe('MAT ECHO PHIA TRUOC', 'SENSOR')
            return

    if d <= STOP_DISTANCE:
        # Dung ngay sau so do gan, roi do xac nhan 1 lan.
        stop()
        mode = 'STOP'
        # Đo xác nhận: nếu đo mới > STOP+3 cm thì loại được một mẫu gần bất thường.
        confirm = measure(2)
        if confirm is not None and confirm > STOP_DISTANCE + 3:
            d = confirm
        elif confirm is not None:
            d = min(d, confirm)

    if d <= STOP_DISTANCE:
        clear_since_ms = None
        print('OBSTACLE |', d, 'cm')
        # Quá gần <=18 cm: lùi ngay một nhịp; nếu đủ khoảng cách: quét.
        state = 'BACK' if d <= TOO_CLOSE_DIST else 'SCAN' 
        return

    change_forward_speed('RUN' if d > SLOW_DISTANCE else 'SLOW')
    # Chỉ reset bộ đếm nếu xe chạy thông suốt và đủ xa vật cản >=1,5 s.
    if recovery_pending:
        if d > RESUME_DISTANCE:
            if clear_since_ms is None:
                clear_since_ms = time.ticks_ms()
            elif time.ticks_diff(time.ticks_ms(), clear_since_ms) >= CLEAR_RUN_MS:
                back_steps = 0
                scan_count = 0
                recovery_pending = False
                clear_since_ms = None
                escape_used = False
                print('RECOVERED | reset bo dem')
        else:
            clear_since_ms = None


# -------------------------------- 5.2 SCAN ---------------------------------
# Mỗi vị trí không quét mãi: scan_count giới hạn số lần xử lý.
# Quét mới hai bên -> tạo choices -> quay thử hoặc lùi.
def step_scan():
    global state, scan_count, choices, choice_idx, escape_turn, escape_used
    escape_turn = False
    if scan_count >= MAX_SCANS:
        enter_safe('HET GIOI HAN QUET', 'LIMIT')
        return
    scan_count += 1
    left, right = scan_sides()
    # Khi ca hai goc mat ECHO, kiem tra cam bien tai GIUA.
    # Gia su phia truoc dang bi chan: neu GIUA van doc duoc thi cam bien
    # co phan hoi; cho thu QUAY voi moi huong, KHONG cho TIEN MÙ.
    # Cả hai góc không có ECHO: chưa đủ căn cứ để coi là hết lối.
    # Kiểm tra giữa; nếu mất ECHO cả ba hướng thì dừng an toàn.
    if left is None and right is None:
        front = measure(2)
        print('SCAN_NO_ECHO | check CENTER:', front)
        if front is None:
            enter_safe('MAT ECHO CA BA HUONG', 'SENSOR')
            return

    # Sắp xếp theo khoảng cách giảm dần, ưu tiên hướng có phép đo.
    choices = turn_options(left, right)
    choice_idx = 0
    if choices:
        print('CHOOSE |', choices,
              '| None = huong CHUA RO, phai verify sau quay')
        state = 'TURN'
    else:
        # Da lui 3 nhip, quet lai SAU KHI lui: neu co ben >=24cm thi thu quay 0.5s.
        # Khong them ham quay thu rieng, tai su dung step_turn() san co.
        # ESCAPE 0,5 s khác rẽ thường: SAU BA NHỊP LÙI, quét MỚI.
        # Nếu 24 <= khoảng cách phía bên < 38, được PHÉP THỬ quay 0,5 s,
        # NHƯNG vẫn phải xác nhận đường trước >40 cm trước khi đi.
        if back_steps >= MAX_BACK_STEPS and not escape_used:
            possible = [(d, direction) for direction, d in
                        (('LEFT', left), ('RIGHT', right))
                        if d is not None and d >= ESCAPE_MIN_SIDE]
            if possible:
                possible.sort(reverse=True)
                choices = [possible[0][1]]
                choice_idx = 0
                escape_turn = True
                escape_used = True
                state = 'TURN'
                print('ESCAPE | scan moi sau lui | quay', choices[0], '0.5s')
                return
        print('NO_ROUTE | hai ben deu bi chan -> BACK')
        state = 'BACK' 


# -------------------------------- 5.3 TURN ---------------------------------
# Thử lần lượt hướng ưu tiên. Quay và đo lại trục trước.
# Nếu thất bại: quay ngược xấp xỉ về hướng gốc, thử hướng tiếp theo.
# CHÚ Ý: 0,5 s chỉ áp dụng khi escape_turn=True sau đủ ba nhịp lùi.
def step_turn():
    global state, choice_idx, mode, recovery_pending, clear_since_ms
    direction = choices[choice_idx]
    duration = ESCAPE_TURN_TIME if escape_turn else TURN_TIME
    print('TURN |', direction, '| try', choice_idx + 1, '/', len(choices))
    rotate(direction, duration)
    time.sleep_ms(120)
    front = measure(3)
    print('VERIFY |', front, 'cm | need >', RESUME_DISTANCE)

    # Điều kiện được TIẾN lại: có số đo hợp lệ và >40 cm.
    if front is not None and front > RESUME_DISTANCE:
        # Chac chan co ECHO o huong vua quay -> moi cho phep tien.
        recovery_pending = True
        clear_since_ms = None
        mode = 'STOP'
        state = 'DRIVE'
        return

    # Khong xac nhan duoc: quay ve huong ban dau truoc khi thu huong khac.
    print('TURN_FAIL | return to original heading')
    rotate(opposite(direction), duration)
    time.sleep_ms(120)
    choice_idx += 1
    if choice_idx < len(choices):
        state = 'TURN'
        return

    # Da thu het ca hai huong. Doc tai GIUA de phan biet cam bien mat
    # ECHO that su voi truong hop 2 huong ben qua thoang/khong phan xa.
    center = measure(2)
    if center is None:
        enter_safe('KHONG XAC NHAN DUOC CAM BIEN SAU QUAY', 'SENSOR')
    else:
        print('TURN_FAIL_ALL | CENTER:', center, '-> BACK va quet vi tri moi')
        state = 'BACK'


# -------------------------------- 5.4 BACK ---------------------------------
# Lùi đúng một nhịp 0,4 s rồi về SCAN; không có while lùi vô hạn.
# Vì không có cảm biến sau, điều kiện ALLOW_BLIND_BACK mang tính
# CẢNH BÁO/QUYẾT ĐỊNH kiểm thử, KHÔNG xác nhận phía sau an toàn.
def step_back():
    global state, mode, back_steps
    if not ALLOW_BLIND_BACK:
        enter_safe('CAM LUI MU: CHUA KIEM TRA VUNG PHIA SAU', 'LIMIT')
        return
    if back_steps >= MAX_BACK_STEPS:
        enter_safe('HET GIOI HAN LUI', 'LIMIT')
        return
    back_steps += 1
    print('BACK |', back_steps, '/', MAX_BACK_STEPS)
    backward()
    time.sleep(BACK_TIME)
    stop()
    mode = 'STOP'
    time.sleep_ms(150)
    state = 'SCAN'  # LUI xong QUET tai vi tri MOI, khong tien vao cho cu.


# -------------------------------- 5.5 SAFE ---------------------------------
# LIMIT: dừng chờ Ctrl+C/reset; SENSOR: định kỳ đo lại để phục hồi.
# Tự phục hồi KHÔNG bỏ qua yêu cầu xác nhận tín hiệu cảm biến.
def step_safe():
    global state, sensor_good, mode
    # SAFE do gioi han / lùi mù: phai dung thuc su, Ctrl+C hoac reset.
    if safe_kind != 'SENSOR':
        time.sleep_ms(500)
        return
    # SAFE mat ECHO: thu kiem tra dinh ky tai GIUA, khong quay/lui mù.
    # Cac lan phuc hoi deu qua vong lap chinh, khong cho vo han trong ham.
    d = measure(2)
    sensor_good = sensor_good + 1 if d is not None else 0
    if sensor_good >= 2:
        print('SENSOR_OK |', d, 'cm | re-evaluate')
        sensor_good = 0
        mode = 'STOP'
        state = 'SCAN' if d <= RESUME_DISTANCE else 'DRIVE'
    else:
        time.sleep_ms(350)


# ============================================================================
# 6. VÒNG LẶP CHƯƠNG TRÌNH VÀ DỪNG KHẨN CẤP
# ============================================================================
# Nguồn duy nhất điều phối chương trình: mỗi lần lặp chạy một hàm step_*.
# Không có các while True lồng nhau trong mỗi trạng thái.
# Thời gian nghỉ 15 ms giữa các bước; trong hàm scan/turn/back sẽ có
# các khoảng chờ cơ khí riêng (servo/motor).
def main():
    stop()
    print('ESP32 FSM SUA SAFE - 5 trang thai')
    print('ALLOW_BLIND_BACK =', ALLOW_BLIND_BACK)
    set_servo_angle(SERVO_CENTER)
    time.sleep(2)
    while True:
        if state == 'DRIVE':
            step_drive()
        elif state == 'SCAN':
            step_scan()
        elif state == 'TURN':
            step_turn()
        elif state == 'BACK':
            step_back()
        else:
            step_safe()
        time.sleep_ms(15)


# Khởi chạy và giải phóng PWM an toàn kể cả khi Ctrl+C hoặc lỗi runtime.
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Dung boi nguoi dung')
    except Exception as e:
        print('LOI:', e)
    finally:
        stop()
        try:
            servo.deinit()
        except Exception:
            pass
        print('DA DUNG TOAN BO DONG CO')
