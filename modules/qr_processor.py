import cv2
import json
import threading
import time
from cv_bridge import CvBridge

class AsyncQRProcessor:
    def __init__(self, config):
        self.config = config
        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()
        
        # config.yaml에서 설정 불러오기
        self.target_width = config['qr_system'].get('process_width', 640)
        self.areas = config['qr_system']['areas']
        self.locations = config['qr_system']['locations']

        # 스레드 공유 변수
        self._current_image = None
        self._lock = threading.Lock()
        self._running = True
        self._latest_result = None  # (x, y, yaw)

        # 백그라운드 워커 스레드 시작
        self._worker_thread = threading.Thread(target=self._process_loop, daemon=True)
        self._worker_thread.start()

    def update_image(self, ros_image_msg):
        """ROS 이미지를 받아서 최신 프레임 갱신 (Non-blocking)"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image_msg, desired_encoding='bgr8')
            # 성능 최적화를 위한 리사이즈
            if cv_image.shape[1] > self.target_width:
                scale = self.target_width / cv_image.shape[1]
                cv_image = cv2.resize(cv_image, None, fx=scale, fy=scale)
            
            with self._lock:
                self._current_image = cv_image
        except Exception as e:
            print(f"[QRProcessor] Image conversion error: {e}")

    def get_result(self):
        """현재까지 인식된 결과 반환"""
        with self._lock:
            return self._latest_result

    def _process_loop(self):
        """무거운 QR 인식 작업을 수행하는 백그라운드 루프"""
        while self._running:
            img_to_process = None
            
            with self._lock:
                if self._current_image is not None:
                    img_to_process = self._current_image.copy()
                    self._current_image = None # 처리 후 비움
            
            if img_to_process is None:
                time.sleep(0.05)
                continue

            try:
                # QR 인식 수행
                data, points, _ = self.qr_detector.detectAndDecode(img_to_process)
                if data:
                    parsed_pose = self._parse_data(data)
                    if parsed_pose:
                        with self._lock:
                            self._latest_result = parsed_pose
            except Exception:
                pass

    def _parse_data(self, data):
        """JSON 문자열 파싱 및 좌표 매핑"""
        try:
            data_list = json.loads(data)
            if len(data_list) <= 1: return None
            area_name = str(data_list[1])

            for region_key, keywords in self.areas.items():
                if area_name in keywords:
                    loc = self.locations[region_key]
                    return (loc['x'], loc['y'], loc['yaw'])
        except:
            pass
        return None