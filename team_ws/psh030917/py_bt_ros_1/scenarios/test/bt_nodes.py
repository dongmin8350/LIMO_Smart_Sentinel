import math
import os
import cv2
import json
os.environ.setdefault('QT_QPA_PLATFORM', 'xcb')
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from modules.base_bt_nodes import BTNodeList, Status, Node, Sequence, Fallback, ReactiveSequence, ReactiveFallback, AlwaysSuccess, AlwaysFailure, Parallel, Repeat
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSAction

try:
    from ultralytics import YOLO
except Exception:
    YOLO = None

# BT Node List
CUSTOM_ACTION_NODES = [
    'CaptureCameraImage',    # 카메라 이미지 캡처 (psh030917)
    'DisplayYOLODetection',  # YOLO 검출 (psh030917)
    'CaptureScreen',        # 화면 캡처 (psh030917)
    'SendTelegramMessage',  # 텔레그램 메시지 전송 (psh030917)
    'StopRobot',            # 로봇 정지 (MinGeun-SMG)
    'PlayLocalAlarm',       # 로컬 알람 재생 (MinGeun-SMG) - 주석 처리됨
    'MoveToCoordinate',     # 특정 좌표로 이동 (psh030917)
    'FollowWaypoints',      # Waypoint 주행 Application (psh030917)
    'RegisterPerson',       # 사람을 등록된 사람 데이터베이스에 추가 (psh030917)
    'ReadFireSensorFromSerial',  # 시리얼 포트에서 화재 센서 읽기 (MinGeun-SMG)
    'PublishFireSensor',          # 화재 센서 데이터를 /fire_sensor 토픽으로 발행 (MinGeun-SMG)
    'ManagePatrolMode'            # 순찰 모드 관리 (han1371)
]

CUSTOM_CONDITION_NODES = [
    'IsDetectedSomething',   # 객체 감지 확인 (psh030917)
    'IsRegisteredPerson',    # 등록된 사람인지 확인 (psh030917)
    'UpdateFireState',       # 화재 상태 업데이트 (MinGeun-SMG)
    'IsFireDetected',        # 화재 감지 여부 확인 (MinGeun-SMG)
    'IsObstacleDetected',    # 장애물 감지 확인 (han1371)
    'IsNightMode',           # 야간 모드 확인 (han1371)
    'IsLidarObstacleDetected'  # LiDAR 기반 장애물 감지 확인 (han1371)
]

# BT Node List
BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)


# Helper function for IoU calculation
def _calculate_iou(box1, box2):
    """
    IoU(Intersection over Union) 계산 함수.
    두 바운딩 박스의 겹침 정도를 0.0 ~ 1.0으로 반환.
    
    Args:
        box1: [x1, y1, x2, y2] 형식의 바운딩 박스
        box2: [x1, y1, x2, y2] 형식의 바운딩 박스
    
    Returns:
        float: IoU 값 (0.0 ~ 1.0)
    """
    # 교집합 영역 계산
    x1_inter = max(box1[0], box2[0])
    y1_inter = max(box1[1], box2[1])
    x2_inter = min(box1[2], box2[2])
    y2_inter = min(box1[3], box2[3])
    
    # 교집합이 없는 경우
    if x2_inter < x1_inter or y2_inter < y1_inter:
        return 0.0
    
    # 교집합 면적
    intersection_area = (x2_inter - x1_inter) * (y2_inter - y1_inter)
    
    # 각 박스의 면적
    box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1])
    box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1])
    
    # 합집합 면적
    union_area = box1_area + box2_area - intersection_area
    
    # IoU 계산
    if union_area == 0:
        return 0.0
    
    iou = intersection_area / union_area
    return iou

class CaptureCameraImage(ConditionWithROSTopics):
    """
    지정된 카메라 토픽을 구독하여 최신 이미지를 blackboard에 저장.
    - 카메라 토픽: /camera/color/image_raw (### 경우에 따라 토픽 변경)
    - blackboard['camera_image']에 최신 이미지 저장
    - 항상 SUCCESS 반환 (이미지가 없으면 RUNNING)
    """
    def __init__(self, name, agent, camera_topic="psh"):  ### 경우에 따라 토픽 변경
        super().__init__(name, agent, [
            (Image, camera_topic, 'camera_image'),
        ])
        self.type = "Action"  # Action으로 사용
        self.camera_topic = camera_topic
        agent.ros_bridge.node.get_logger().info(f'CaptureCameraImage: 구독 시작 - 토픽: {camera_topic}')
    
    def _predicate(self, agent, blackboard) -> bool:
        """이미지가 있으면 blackboard에 저장하고 SUCCESS"""
        cache = self._cache
        if "camera_image" in cache:
            # 최신 이미지를 blackboard에 저장
            blackboard['camera_image'] = cache["camera_image"]
            agent.ros_bridge.node.get_logger().debug(f'CaptureCameraImage: 이미지 수신됨 (토픽: {self.camera_topic})')
            return True
        else:
            # 이미지가 없으면 blackboard에서 제거 (이전 이미지가 남아있지 않도록)
            if blackboard.get('camera_image') is not None:
                agent.ros_bridge.node.get_logger().warning(f'CaptureCameraImage: 이미지 수신 실패 - 토픽 "{self.camera_topic}"에서 메시지를 받지 못함. blackboard 초기화.')
            blackboard['camera_image'] = None
            return False

# 모듈 레벨 변수로 초기화 상태 관리 (ReactiveSequence가 halt()를 매 틱마다 호출하므로 모듈 변수 사용)
_display_yolo_initialized = False
_display_yolo_publisher = None
_display_yolo_yolo_model = None
_display_yolo_window_created = False

class DisplayYOLODetection(Node):
    """
    YOLO 검출을 수행하고 결과를 ROS 토픽으로 퍼블리시하는 노드.
    - blackboard['camera_image']에서 이미지 가져오기
    - YOLO 모델 경로: blackboard['yolo_model_path'] 또는 기본값 'yolov10n.pt'
    - ROS 토픽: blackboard['output_topic'] 또는 기본값 '/yolo/image' (### 경우에 따라 토픽 변경)
    - 매 틱마다 카메라 이미지를 읽어 YOLO 검출 후 ROS 토픽으로 퍼블리시
    """
    
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self._bridge = CvBridge()
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        global _display_yolo_initialized, _display_yolo_publisher
        global _display_yolo_yolo_model, _display_yolo_window_created
        
        # 설정 가져오기
        model_path = blackboard.get('yolo_model_path', 'yolov8n.pt')
        output_topic = blackboard.get('output_topic', '/yolo/image')  ### 경우에 따라 토픽 변경
        
        # 초기화 (최초 1회만)
        if not _display_yolo_initialized:
            if not self._initialize(model_path, output_topic):
                self.status = Status.FAILURE
                return self.status
        
        # blackboard에서 카메라 이미지 가져오기
        camera_image_msg = blackboard.get('camera_image')
        if camera_image_msg is None:
            self.ros.node.get_logger().warning('No camera image in blackboard yet - DisplayYOLODetection cannot display')
            self.status = Status.RUNNING
            return self.status
        
        # ROS Image 메시지를 OpenCV 이미지로 변환
        try:
            frame = self._bridge.imgmsg_to_cv2(camera_image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.ros.node.get_logger().error(f'Failed to convert image: {e}')
            self.status = Status.FAILURE
            return self.status
        
        # YOLO 검출 및 퍼블리시
        try:
            # YOLO 검출 수행
            results = _display_yolo_yolo_model(frame, verbose=False)
            res = results[0]
            
            # 검출 결과 파싱
            detections = self._parse_detections(res)
            
            # blackboard에 검출 결과 저장
            annotated_frame = res.plot()
            blackboard['yolo_detections'] = detections
            blackboard['current_frame'] = frame
            blackboard['annotated_frame'] = annotated_frame
            
            # OpenCV 윈도우에 표시
            try:
                cv2.imshow("YOLO Detection", annotated_frame)
                cv2.waitKey(1)
                self.ros.node.get_logger().debug(f'Displayed frame in OpenCV window: shape={annotated_frame.shape}')
            except Exception as e:
                self.ros.node.get_logger().error(f'Failed to display OpenCV window: {e}')
            
            # ROS 토픽으로 퍼블리시
            self._publish_frame(annotated_frame)
            
            self.status = Status.SUCCESS
            return self.status
            
        except Exception as e:
            self.ros.node.get_logger().error(f'YOLO processing failed: {e}')
            import traceback
            self.ros.node.get_logger().error(f'Traceback: {traceback.format_exc()}')
            self.status = Status.FAILURE
            return self.status
    
    def _initialize(self, model_path, output_topic):
        """초기화 로직 (최초 1회만 실행)"""
        global _display_yolo_initialized, _display_yolo_publisher
        global _display_yolo_yolo_model, _display_yolo_window_created
        
        try:
            # YOLO 모델 로드
            if YOLO is None:
                self.ros.node.get_logger().error('ultralytics YOLO package not found. Install `ultralytics` to use this node.')
                return False
            
            # GPU 모드로 실행 (CUDA 사용)
            _display_yolo_yolo_model = YOLO(model_path)
            self.ros.node.get_logger().info(f'YOLO model loaded: {model_path}')
            
            # ROS 퍼블리셔 생성
            _display_yolo_publisher = self.ros.node.create_publisher(Image, output_topic, 10)
            
            # OpenCV 윈도우 생성
            try:
                cv2.namedWindow("YOLO Detection", cv2.WINDOW_NORMAL)
                _display_yolo_window_created = True
                self.ros.node.get_logger().info('OpenCV window "YOLO Detection" created successfully')
            except Exception as e:
                self.ros.node.get_logger().error(f'Failed to create OpenCV window: {e}')
                return False
            
            _display_yolo_initialized = True
            self.ros.node.get_logger().info(f'YOLO Detection initialized: model={model_path}, topic={output_topic}')
            return True
            
        except Exception as e:
            self.ros.node.get_logger().error(f'Initialization failed: {e}')
            return False
    
    def _parse_detections(self, res):
        """YOLO 검출 결과를 파싱하여 리스트로 반환"""
        detections = []
        if hasattr(res, 'boxes') and res.boxes is not None:
            try:
                boxes = res.boxes.xyxy.cpu().numpy()
                confs = res.boxes.conf.cpu().numpy()
                cls_inds = res.boxes.cls.cpu().numpy().astype(int)
                
                for box, conf, cls_i in zip(boxes, confs, cls_inds):
                    class_name = _display_yolo_yolo_model.names[int(cls_i)] if hasattr(_display_yolo_yolo_model, 'names') else str(int(cls_i))
                    detections.append({
                        'class': class_name,
                        'confidence': float(conf),
                        'bbox': [float(box[0]), float(box[1]), float(box[2]), float(box[3])]
                    })
            except Exception as e:
                self.ros.node.get_logger().error(f'Failed to parse detections: {e}')
        return detections
    
    def _publish_frame(self, annotated_frame):
        """검출된 프레임을 ROS 토픽으로 퍼블리시"""
        try:
            msg = self._bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            msg.header.stamp = self.ros.node.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            _display_yolo_publisher.publish(msg)
        except Exception as e:
            self.ros.node.get_logger().error(f'Failed to publish: {e}')
    
    def halt(self):
        """노드 중지 시 정리 (ReactiveSequence가 매 틱마다 호출하므로 초기화 플래그는 유지)"""
        # ReactiveSequence가 매 틱마다 halt()를 호출하므로
        # 여기서 초기화 플래그를 리셋하면 안 됩니다.
        pass


# IsDetectedSomething 노드의 이전 바운딩박스 및 마지막 감지 시간 추적
_is_detected_something_previous_boxes = {}  # {node_name: [(bbox, timestamp), ...]}
_is_detected_something_last_detection = {}  # {node_name: timestamp}

# Chroma를 사용한 사람 재식별을 위한 전역 변수
try:
    import chromadb
    from chromadb.config import Settings
    CHROMA_AVAILABLE = True
except ImportError:
    CHROMA_AVAILABLE = False
    chromadb = None

_chroma_clients = {}  # {node_name: chromadb.Client}
_chroma_collections = {}  # {node_name: chromadb.Collection}
_registered_person_collections = {}  # {node_name: chromadb.Collection} - 등록된 사람용


class IsDetectedSomething(Node):
    """
    blackboard에서 YOLO 검출 결과를 확인하여 새로운 객체가 감지되었는지 확인.
    - 감지할 클래스: XML 파라미터 'target_class' (기본값: "person")
    - blackboard['yolo_detections']에서 지정된 클래스 검색
    - confidence threshold: XML 파라미터 'confidence_threshold' 또는 기본값 0.5
    - IoU 기반 중복 감지: XML 파라미터 'iou_threshold' (기본값: 0.3)
    - 시간 기반 쿨다운: XML 파라미터 'cooldown_time' (기본값: 10.0초)
    """
    
    def __init__(self, name, agent, target_class="person", confidence_threshold=0.5, 
                 cooldown_time=10.0, iou_threshold=0.3):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.target_class = target_class.lower()  # 소문자로 변환하여 비교
        self.default_confidence_threshold = confidence_threshold
        self.cooldown_time = float(cooldown_time)  # 쿨다운 시간 (초)
        self.iou_threshold = float(iou_threshold)  # IoU 임계값
        self.type = "Condition"
        
        agent.ros_bridge.node.get_logger().info(
            f'IsDetectedSomething: 감지 대상 = "{self.target_class}", '
            f'confidence = {confidence_threshold}, '
            f'cooldown = {cooldown_time}초, '
            f'iou_threshold = {iou_threshold}'
        )
    
    async def run(self, agent, blackboard):
        import time
        global _is_detected_something_previous_boxes
        global _is_detected_something_last_detection
        
        # 검출 결과 가져오기
        detections = blackboard.get('yolo_detections', [])
        
        # confidence threshold 가져오기 (blackboard 우선, 없으면 기본값)
        threshold_key = f'{self.target_class}_confidence_threshold'
        threshold = blackboard.get(threshold_key, self.default_confidence_threshold)
        
        # 현재 프레임의 객체 필터링 (confidence threshold 이상만)
        detected_objects = []
        for detection in detections:
            class_name = detection.get('class', '').lower()
            if class_name == self.target_class:
                confidence = detection.get('confidence', 0.0)
                if confidence >= threshold:
                    detected_objects.append(detection)
        
        # 감지된 객체가 없으면 FAILURE
        if not detected_objects:
            self.status = Status.FAILURE
            return self.status
        
        # 현재 시간
        current_time = time.time()
        
        # 이전 바운딩박스 목록 가져오기
        if self.name not in _is_detected_something_previous_boxes:
            _is_detected_something_previous_boxes[self.name] = []
        
        previous_boxes = _is_detected_something_previous_boxes[self.name]
        
        # 각 감지된 객체에 대해 새로운 객체인지 확인
        new_objects = []
        for obj in detected_objects:
            bbox = obj.get('bbox')
            if not bbox:
                continue
            
            # 이전 바운딩박스들과 IoU 비교
            is_new = True
            for prev_bbox, prev_time in previous_boxes:
                iou = _calculate_iou(bbox, prev_bbox)
                if iou > self.iou_threshold:
                    # 같은 물체로 판단 (IoU가 임계값보다 높음)
                    is_new = False
                    self.ros.node.get_logger().debug(
                        f'{self.target_class}: 같은 물체 감지됨 (IoU: {iou:.2f})'
                    )
                    break
            
            if is_new:
                new_objects.append(obj)
        
        # 새로운 객체가 없으면 FAILURE
        if not new_objects:
            self.ros.node.get_logger().debug(
                f'{self.target_class}: 모두 기존 물체임 (IoU 기반 중복 감지)'
            )
            self.status = Status.FAILURE
            return self.status
        
        # 쿨다운 시간 체크
        last_detection_time = _is_detected_something_last_detection.get(self.name, 0)
        time_since_last = current_time - last_detection_time
        
        if time_since_last < self.cooldown_time:
            # 쿨다운 시간이 아직 안 지남
            self.ros.node.get_logger().debug(
                f'{self.target_class}: 쿨다운 대기 중 ({time_since_last:.1f}/{self.cooldown_time}초)'
            )
            self.status = Status.FAILURE
            return self.status
        
        # 새로운 객체이고 쿨다운도 지남 -> SUCCESS
        # 가장 높은 confidence인 객체 선택
        best_object = max(new_objects, key=lambda x: x.get('confidence', 0.0))
        best_bbox = best_object.get('bbox')
        
        # blackboard에 저장
        blackboard[f'detected_{self.target_class}'] = best_object
        blackboard[f'{self.target_class}_confidence'] = best_object.get('confidence', 0.0)
        
        # 이전 바운딩박스 목록 업데이트
        _is_detected_something_previous_boxes[self.name].append((best_bbox, current_time))
        
        # 오래된 바운딩박스 제거 (쿨다운 시간의 2배 이상 지난 것)
        cutoff_time = current_time - (self.cooldown_time * 2)
        _is_detected_something_previous_boxes[self.name] = [
            (bbox, timestamp) for bbox, timestamp in _is_detected_something_previous_boxes[self.name]
            if timestamp > cutoff_time
        ]
        
        # 마지막 감지 시간 업데이트
        _is_detected_something_last_detection[self.name] = current_time
        
        self.ros.node.get_logger().info(
            f'새로운 {self.target_class} 감지! (신뢰도: {best_object.get("confidence", 0.0):.2f}, '
            f'쿨다운 경과: {time_since_last:.1f}초)'
        )
        
        self.status = Status.SUCCESS
        return self.status


class IsRegisteredPerson(Node):
    """
    감지된 사람이 미리 등록된 사람인지 확인하는 노드.
    - blackboard['detected_person'] 또는 blackboard['detected_{target_class}']에서 감지된 사람 정보 가져오기
    - 등록된 사람 데이터베이스(Chroma)에서 유사도 검색
    - 유사도 임계값: XML 파라미터 'similarity_threshold' (기본값: 0.5, 낮은 각도에 맞게 조정)
    - 여러 프레임 평균: XML 파라미터 'use_multi_frame' (기본값: True)
    - 상위 후보 검색: XML 파라미터 'top_k' (기본값: 3)
    - 등록된 사람이면 SUCCESS, 아니면 FAILURE
    - blackboard에 'is_registered_person' (bool) 저장
    """
    def __init__(self, name, agent, target_class="person", similarity_threshold=0.5, 
                 use_multi_frame=True, top_k=3, frame_buffer_size=5):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.target_class = target_class.lower()
        self.similarity_threshold = float(similarity_threshold)  # 낮은 각도에 맞게 기본값 낮춤
        self.use_multi_frame = bool(use_multi_frame)
        self.top_k = int(top_k)  # 상위 K개 후보 검색
        self.frame_buffer_size = int(frame_buffer_size)  # 프레임 버퍼 크기
        self.type = "Condition"
        
        # 프레임 버퍼 (여러 프레임의 임베딩 저장)
        self._frame_embeddings = []
        self._frame_timestamps = []
        
        # 등록된 사람용 Chroma 초기화
        if CHROMA_AVAILABLE:
            self._init_registered_person_collection()
        else:
            self.ros.node.get_logger().error(
                'ChromaDB가 설치되지 않았습니다. IsRegisteredPerson 노드를 사용할 수 없습니다.'
            )
        
        agent.ros_bridge.node.get_logger().info(
            f'IsRegisteredPerson: 초기화 완료 - target_class="{self.target_class}", '
            f'similarity_threshold={similarity_threshold}, '
            f'use_multi_frame={use_multi_frame}, top_k={top_k}'
        )
    
    def _init_registered_person_collection(self):
        """등록된 사람용 Chroma 컬렉션 초기화"""
        global _chroma_clients, _registered_person_collections
        
        try:
            # 새로운 Chroma 클라이언트 생성 방식
            if self.name not in _chroma_clients:
                # PersistentClient 사용 (새로운 방식)
                _chroma_clients[self.name] = chromadb.PersistentClient(
                    path=f"./chroma_db_registered_{self.name}"
                )
            
            # 등록된 사람용 컬렉션 생성 또는 가져오기
            collection_name = f"registered_{self.target_class}_{self.name}"
            _registered_person_collections[self.name] = _chroma_clients[self.name].get_or_create_collection(
                name=collection_name,
                metadata={"description": f"Registered {self.target_class} embeddings"}
            )
            
            # 등록된 사람 수 확인
            count = _registered_person_collections[self.name].count()
            self.ros.node.get_logger().info(
                f'등록된 사람 컬렉션 초기화 완료: "{collection_name}" (등록된 사람 수: {count})'
            )
        except Exception as e:
            self.ros.node.get_logger().error(f'등록된 사람 컬렉션 초기화 실패: {e}')
    
    def _extract_person_image(self, frame, bbox, use_upper_body=False, focus_top_half=False, focus_bottom_half=True):
        """
        바운딩박스 영역에서 사람 이미지 추출
        - use_upper_body=True: 상체 영역만 추출
        - focus_top_half=True: 상단 50%만 사용
        - focus_bottom_half=True: 하체 50%만 사용 (낮은 각도에서 하체가 더 일관된 특징 제공)
        """
        x1, y1, x2, y2 = map(int, bbox)
        h, w = frame.shape[:2]
        x1 = max(0, min(x1, w))
        y1 = max(0, min(y1, h))
        x2 = max(0, min(x2, w))
        y2 = max(0, min(y2, h))
        
        person_img = frame[y1:y2, x1:x2]
        
        if person_img.size == 0:
            return person_img
        
        img_h = person_img.shape[0]
        
        # 낮은 각도에서는 하체가 더 일관된 특징을 제공 (바지, 신발 등)
        if focus_bottom_half:
            # 하체 50%만 사용 (바지, 신발 등)
            bottom_half_start = int(img_h * 0.5)
            person_img = person_img[bottom_half_start:, :]
        elif focus_top_half:
            # 상단 50%만 사용 (얼굴 + 상체 상단)
            top_half_end = int(img_h * 0.5)
            person_img = person_img[:top_half_end, :]
        elif use_upper_body:
            # 상체 60%만 사용 (얼굴 + 상체)
            upper_body_end = int(img_h * 0.6)
            person_img = person_img[:upper_body_end, :]
        
        return person_img
    
    def _get_image_embedding(self, image, use_enhanced_features=True):
        """
        이미지를 임베딩 벡터로 변환 (개선된 방법)
        - use_enhanced_features=True: 히스토그램, 색상 특징 등 추가
        """
        import numpy as np
        
        if image.size == 0:
            return None
        
        # 이미지 전처리
        # 1. 밝기 정규화 (조명 변화에 강건하게)
        if len(image.shape) == 3:
            lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            l_channel = lab[:, :, 0]
            # CLAHE (Contrast Limited Adaptive Histogram Equalization) 적용
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            l_channel = clahe.apply(l_channel)
            lab[:, :, 0] = l_channel
            image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        
        # 2. 이미지 리사이즈 (더 큰 크기로 - 특징 보존)
        resized = cv2.resize(image, (256, 256))
        
        # 3. RGB로 변환
        if len(resized.shape) == 3 and resized.shape[2] == 3:
            resized_rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        else:
            resized_rgb = resized
        
        # 기본 임베딩: 이미지 픽셀 값
        pixel_features = resized_rgb.flatten().astype(np.float32)
        
        if use_enhanced_features:
            # 추가 특징 추출
            features_list = [pixel_features]
            
            # 4. 색상 히스토그램 특징 (옷 색상 등)
            hist_b = cv2.calcHist([resized], [0], None, [32], [0, 256])
            hist_g = cv2.calcHist([resized], [1], None, [32], [0, 256])
            hist_r = cv2.calcHist([resized], [2], None, [32], [0, 256])
            hist_features = np.concatenate([hist_b.flatten(), hist_g.flatten(), hist_r.flatten()]).astype(np.float32)
            features_list.append(hist_features)
            
            # 5. 텍스처 특징 (LBP - Local Binary Pattern)
            gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY) if len(resized.shape) == 3 else resized
            # 간단한 LBP 구현
            lbp = self._simple_lbp(gray)
            lbp_hist = cv2.calcHist([lbp], [0], None, [256], [0, 256]).flatten().astype(np.float32)
            features_list.append(lbp_hist)
            
            # 6. 하체 영역 강조 (하단 40% 영역의 특징 가중치 증가 - 바지, 신발 등)
            bottom_region = resized_rgb[int(resized_rgb.shape[0] * 0.6):, :]
            bottom_features = bottom_region.flatten().astype(np.float32)
            features_list.append(bottom_features * 1.5)  # 가중치 증가
            
            # 모든 특징 결합
            embedding = np.concatenate(features_list).astype(np.float32)
        else:
            embedding = pixel_features
        
        # L2 정규화
        embedding = embedding / (np.linalg.norm(embedding) + 1e-8)
        
        return embedding.tolist()
    
    def _simple_lbp(self, image):
        """간단한 LBP (Local Binary Pattern) 구현"""
        import numpy as np
        h, w = image.shape
        lbp = np.zeros_like(image)
        
        for i in range(1, h-1):
            for j in range(1, w-1):
                center = image[i, j]
                code = 0
                code |= (image[i-1, j-1] >= center) << 7
                code |= (image[i-1, j] >= center) << 6
                code |= (image[i-1, j+1] >= center) << 5
                code |= (image[i, j+1] >= center) << 4
                code |= (image[i+1, j+1] >= center) << 3
                code |= (image[i+1, j] >= center) << 2
                code |= (image[i+1, j-1] >= center) << 1
                code |= (image[i, j-1] >= center) << 0
                lbp[i, j] = code
        
        return lbp
    
    async def run(self, agent, blackboard):
        global _registered_person_collections
        
        # 감지된 사람 정보 가져오기
        detected_person = blackboard.get(f'detected_{self.target_class}')
        if detected_person is None:
            # 하위 호환성: detected_person도 확인
            detected_person = blackboard.get('detected_person')
        
        if detected_person is None:
            self.ros.node.get_logger().debug('감지된 사람 정보가 없습니다.')
            blackboard['is_registered_person'] = False
            self.status = Status.FAILURE
            return self.status
        
        # 현재 프레임 가져오기
        current_frame = blackboard.get('current_frame')
        if current_frame is None:
            self.ros.node.get_logger().debug('현재 프레임이 없습니다.')
            blackboard['is_registered_person'] = False
            self.status = Status.FAILURE
            return self.status
        
        # 바운딩박스 가져오기
        bbox = detected_person.get('bbox')
        if not bbox:
            self.ros.node.get_logger().debug('바운딩박스 정보가 없습니다.')
            blackboard['is_registered_person'] = False
            self.status = Status.FAILURE
            return self.status
        
        # 등록된 사람 컬렉션이 없으면 실패
        if self.name not in _registered_person_collections:
            self.ros.node.get_logger().warn('등록된 사람 컬렉션이 초기화되지 않았습니다.')
            blackboard['is_registered_person'] = False
            self.status = Status.FAILURE
            return self.status
        
        import time
        import numpy as np
        
        # 여러 프레임 평균 사용 여부에 따라 이미지 추출
        # 낮은 각도에서는 하체가 더 일관된 특징을 제공 (바지, 신발 등)
        person_img = self._extract_person_image(current_frame, bbox, focus_bottom_half=True)
        if person_img.size == 0:
            self.ros.node.get_logger().debug('사람 이미지 추출 실패')
            blackboard['is_registered_person'] = False
            self.status = Status.FAILURE
            return self.status
        
        embedding = self._get_image_embedding(person_img, use_enhanced_features=True)
        if embedding is None:
            self.ros.node.get_logger().debug('임베딩 생성 실패')
            blackboard['is_registered_person'] = False
            self.status = Status.FAILURE
            return self.status
        
        # 여러 프레임 평균 사용
        current_time = time.time()
        if self.use_multi_frame:
            # 프레임 버퍼에 추가
            self._frame_embeddings.append(embedding)
            self._frame_timestamps.append(current_time)
            
            # 오래된 프레임 제거 (5초 이상 지난 것)
            cutoff_time = current_time - 5.0
            valid_indices = [i for i, ts in enumerate(self._frame_timestamps) if ts > cutoff_time]
            self._frame_embeddings = [self._frame_embeddings[i] for i in valid_indices]
            self._frame_timestamps = [self._frame_timestamps[i] for i in valid_indices]
            
            # 최대 버퍼 크기 제한
            if len(self._frame_embeddings) > self.frame_buffer_size:
                self._frame_embeddings = self._frame_embeddings[-self.frame_buffer_size:]
                self._frame_timestamps = self._frame_timestamps[-self.frame_buffer_size:]
            
            # 평균 임베딩 계산 (최소 2개 프레임 이상일 때만)
            if len(self._frame_embeddings) >= 2:
                avg_embedding = np.mean(self._frame_embeddings, axis=0).tolist()
                # L2 정규화
                avg_embedding = (np.array(avg_embedding) / (np.linalg.norm(avg_embedding) + 1e-8)).tolist()
                query_embedding = avg_embedding
                self.ros.node.get_logger().debug(
                    f'여러 프레임 평균 사용: {len(self._frame_embeddings)}개 프레임'
                )
            else:
                query_embedding = embedding
        else:
            query_embedding = embedding
        
        # 등록된 사람 데이터베이스에서 검색
        try:
            collection = _registered_person_collections[self.name]
            
            # 등록된 사람이 없으면 실패
            if collection.count() == 0:
                self.ros.node.get_logger().debug('등록된 사람이 없습니다.')
                blackboard['is_registered_person'] = False
                self.status = Status.FAILURE
                return self.status
            
            # 상위 K개 후보 검색 (여러 후보의 평균 유사도 사용)
            results = collection.query(
                query_embeddings=[query_embedding],
                n_results=self.top_k  # 상위 K개 후보
            )
            
            if results['distances'] and len(results['distances'][0]) > 0:
                distances = results['distances'][0]
                
                # 여러 후보의 평균 유사도 계산 (가중 평균: 가장 유사한 것에 더 높은 가중치)
                similarities = [1.0 - (d / 2.0) for d in distances]
                
                # 가중 평균 계산 (유사도가 높을수록 더 높은 가중치)
                weights = [max(0, s) for s in similarities]  # 음수 가중치 방지
                if sum(weights) > 0:
                    weighted_similarity = sum(s * w for s, w in zip(similarities, weights)) / sum(weights)
                else:
                    weighted_similarity = similarities[0] if similarities else 0.0
                
                # 최고 유사도도 저장
                max_similarity = max(similarities) if similarities else 0.0
                
                # 평균 유사도 또는 최고 유사도가 임계값 이상이면 등록된 사람으로 판단
                final_similarity = max(weighted_similarity, max_similarity * 0.9)  # 최고 유사도도 고려하되 약간 낮춤
                
                if final_similarity >= self.similarity_threshold:
                    # 등록된 사람으로 판단 (가장 유사한 후보 선택)
                    best_idx = similarities.index(max_similarity)
                    matched_id = results['ids'][0][best_idx] if results['ids'] and len(results['ids'][0]) > best_idx else None
                    matched_metadata = results['metadatas'][0][best_idx] if results['metadatas'] and len(results['metadatas'][0]) > best_idx else {}
                    
                    blackboard['is_registered_person'] = True
                    blackboard['matched_person_id'] = matched_id
                    blackboard['matched_person_name'] = matched_metadata.get('name', 'Unknown')
                    blackboard['person_similarity'] = final_similarity
                    blackboard['person_max_similarity'] = max_similarity
                    blackboard['person_weighted_similarity'] = weighted_similarity
                    
                    self.ros.node.get_logger().info(
                        f'등록된 사람 확인됨! (이름: {matched_metadata.get("name", "Unknown")}, '
                        f'최종 유사도: {final_similarity:.2f}, 최고: {max_similarity:.2f}, '
                        f'가중 평균: {weighted_similarity:.2f}, 임계값: {self.similarity_threshold})'
                    )
                    
                    self.status = Status.SUCCESS
                    return self.status
                else:
                    # 유사도가 낮음
                    blackboard['is_registered_person'] = False
                    blackboard['person_similarity'] = final_similarity
                    blackboard['person_max_similarity'] = max_similarity
                    
                    self.ros.node.get_logger().debug(
                        f'등록되지 않은 사람 (최종 유사도: {final_similarity:.2f} < {self.similarity_threshold}, '
                        f'최고: {max_similarity:.2f})'
                    )
                    
                    self.status = Status.FAILURE
                    return self.status
            else:
                # 검색 결과 없음
                blackboard['is_registered_person'] = False
                self.ros.node.get_logger().debug('등록된 사람 중 일치하는 사람 없음')
                self.status = Status.FAILURE
                return self.status
                
        except Exception as e:
            import traceback
            self.ros.node.get_logger().error(f'등록된 사람 검색 실패: {e}')
            self.ros.node.get_logger().error(traceback.format_exc())
            blackboard['is_registered_person'] = False
            self.status = Status.FAILURE
            return self.status


class CaptureScreen(Node):
    """
    현재 프레임을 이미지 파일로 저장하고 선택적으로 텔레그램으로 전송.
    - blackboard['annotated_frame'] 또는 blackboard['current_frame']에서 프레임 가져오기
    - 저장 경로: blackboard['save_path'] 또는 기본값 'scenarios/test/captured_images/'
    - 타임스탬프를 포함한 파일명으로 저장
    - 텔레그램 전송: XML 파라미터 'send_to_telegram' (기본값: False)
    - 텔레그램 캡션: XML 파라미터 'telegram_caption' 또는 blackboard['telegram_image_caption']
    """
    def __init__(self, name, agent, send_to_telegram=False, telegram_caption=""):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.type = "Action"
        self.send_to_telegram = bool(send_to_telegram)
        self.default_telegram_caption = telegram_caption
        
        # 저장 디렉토리 기본값
        self.default_save_dir = os.path.join(
            os.path.dirname(__file__), 'captured_images'
        )
        
        # 텔레그램 전송을 위한 퍼블리셔 (필요할 때만 생성)
        self._telegram_image_publisher = None
        if self.send_to_telegram:
            self._telegram_image_publisher = self.ros.node.create_publisher(Image, '/image_from_ros', 10)
            self._bridge = CvBridge()
            agent.ros_bridge.node.get_logger().info(
                f'CaptureScreen: 텔레그램 전송 활성화 - 캡션: "{telegram_caption}"'
            )
    
    async def run(self, agent, blackboard):
        # 저장 디렉토리 설정
        save_dir = blackboard.get('save_path', self.default_save_dir)
        
        # 디렉토리가 없으면 생성
        os.makedirs(save_dir, exist_ok=True)
        
        # 프레임 가져오기 (annotated_frame 우선, 없으면 current_frame)
        frame = blackboard.get('annotated_frame')
        if frame is None:
            frame = blackboard.get('current_frame')
        
        if frame is None:
            self.ros.node.get_logger().warn('No frame available in blackboard for capture')
            self.status = Status.FAILURE
            return self.status
        
        try:
            # 타임스탬프로 파일명 생성
            from datetime import datetime
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # 밀리초 포함
            
            # 감지된 객체 정보를 동적으로 가져오기
            # IsDetectedSomething에서 설정한 detected_{target_class} 키를 찾음
            detected_object = None
            object_class = None
            
            # blackboard에서 detected_* 패턴의 키를 찾음
            for key in blackboard.keys():
                if key.startswith('detected_') and key != 'detected_person':
                    detected_object = blackboard.get(key)
                    object_class = key.replace('detected_', '')
                    break
            
            # detected_person도 확인 (하위 호환성)
            if detected_object is None:
                detected_object = blackboard.get('detected_person')
                if detected_object:
                    object_class = 'person'
            
            # 파일명 생성
            if detected_object and object_class:
                confidence = detected_object.get('confidence', 0.0)
                filename = os.path.join(save_dir, f"captured_{object_class}_{confidence:.2f}_{timestamp}.jpg")
                self.ros.node.get_logger().info(
                    f'CaptureScreen: {object_class} 감지 (신뢰도: {confidence:.2f}) - 스크린샷 저장'
                )
            else:
                filename = os.path.join(save_dir, f"captured_frame_{timestamp}.jpg")
                self.ros.node.get_logger().info('CaptureScreen: 일반 프레임 저장')
            
            # 이미지 파일로 저장
            cv2.imwrite(filename, frame)
            
            # 저장된 파일 경로를 blackboard에 기록
            blackboard['last_captured_image'] = filename
            blackboard['capture_result'] = 'succeeded'
            
            self.ros.node.get_logger().info(f'Image saved: {filename}')
            
            # 텔레그램 전송 (옵션)
            send_telegram = blackboard.get('send_capture_to_telegram', self.send_to_telegram)
            if send_telegram:
                try:
                    # OpenCV 이미지를 ROS Image 메시지로 변환
                    image_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    
                    # 캡션 가져오기 (blackboard 우선, 없으면 기본값)
                    caption = blackboard.get('telegram_image_caption', self.default_telegram_caption)
                    if not caption:
                        # 기본 캡션 생성
                        if detected_object and object_class:
                            caption = f"{object_class} 감지됨 (신뢰도: {confidence:.2f})"
                        else:
                            caption = "이미지 캡처됨"
                    
                    # frame_id에 캡션 설정 (텔레그램 브리지가 frame_id를 캡션으로 사용)
                    image_msg.header.frame_id = str(caption)
                    image_msg.header.stamp = self.ros.node.get_clock().now().to_msg()
                    
                    # 텔레그램으로 전송
                    if self._telegram_image_publisher is None:
                        self._telegram_image_publisher = self.ros.node.create_publisher(Image, '/image_from_ros', 10)
                        self._bridge = CvBridge()
                    
                    self._telegram_image_publisher.publish(image_msg)
                    self.ros.node.get_logger().info(f'CaptureScreen: 텔레그램으로 이미지 전송 완료 - "{caption}"')
                except Exception as e:
                    self.ros.node.get_logger().error(f'CaptureScreen: 텔레그램 전송 실패 - {e}')
                    # 텔레그램 전송 실패해도 저장은 성공으로 처리
            
            self.status = Status.SUCCESS
            return self.status
            
        except Exception as e:
            self.ros.node.get_logger().error(f'Failed to save image: {e}')
            self.status = Status.FAILURE
            return self.status


# SendTelegramMessage 노드의 마지막 전송 시간 추적 (노드별로 관리)
_send_telegram_last_sent = {}  # {node_name: last_sent_time}

class SendTelegramMessage(Node):
    """
    텔레그램으로 메시지를 전송하는 노드.
    - 메시지: XML 파라미터 'message' 또는 blackboard['telegram_message']
    - 주기: XML 파라미터 'interval' (초 단위, 기본값: 0 = 매번 전송)
    - ROS 토픽: '/message_from_ros' (std_msgs/String)
    - telegram_ros2_bridge가 실행 중이어야 함
    """
    def __init__(self, name, agent, message="사람이 감지되었습니다.", interval=0.0):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.default_message = message
        self.interval = float(interval)  # 초 단위
        self.type = "Action"
        
        # 텔레그램 브리지 토픽으로 퍼블리셔 생성
        from std_msgs.msg import String
        self._publisher = self.ros.node.create_publisher(String, '/message_from_ros', 10)
        agent.ros_bridge.node.get_logger().info(
            f'SendTelegramMessage: 초기화 완료 - 기본 메시지: "{message}", 주기: {interval}초'
        )
    
    async def run(self, agent, blackboard):
        import time
        
        global _send_telegram_last_sent
        
        # 메시지 가져오기 (blackboard 우선, 없으면 기본값)
        message = blackboard.get('telegram_message', self.default_message)
        
        # 주기 가져오기 (blackboard 우선, 없으면 기본값)
        interval = blackboard.get('telegram_interval', self.interval)
        
        if not message:
            self.ros.node.get_logger().warn('SendTelegramMessage: 전송할 메시지가 없습니다')
            self.status = Status.FAILURE
            return self.status
        
        # 주기 확인
        current_time = time.time()
        last_sent_time = _send_telegram_last_sent.get(self.name, 0)
        
        if interval > 0 and (current_time - last_sent_time) < interval:
            # 아직 주기 시간이 지나지 않음
            self.ros.node.get_logger().debug(
                f'SendTelegramMessage: 주기 대기 중 ({current_time - last_sent_time:.1f}/{interval}초)'
            )
            self.status = Status.SUCCESS
            return self.status
        
        # 메시지 전송
        try:
            from std_msgs.msg import String
            msg = String()
            msg.data = str(message)
            
            self._publisher.publish(msg)
            _send_telegram_last_sent[self.name] = current_time
            
            if interval > 0:
                self.ros.node.get_logger().info(
                    f'SendTelegramMessage: 텔레그램 메시지 전송 - "{message}" (다음 전송: {interval}초 후)'
                )
            else:
                self.ros.node.get_logger().info(f'SendTelegramMessage: 텔레그램 메시지 전송 - "{message}"')
            
            self.status = Status.SUCCESS
            return self.status
            
        except Exception as e:
            self.ros.node.get_logger().error(f'SendTelegramMessage: 메시지 전송 실패 - {e}')
            self.status = Status.FAILURE
            return self.status


class UpdateFireState(ConditionWithROSTopics):
    """
    화재 센서 데이터를 읽어 blackboard에 상태를 업데이트하는 노드.
    - 토픽: XML 파라미터 'fire_sensor_topic' 또는 기본값 "/fire_sensor"
    - JSON 형식: {"flame": 1, "temp": 600}
    - 화재 감지 기준:
      - flame_threshold: XML 파라미터 (기본값: 1) - flame >= 이 값이면 화재
      - flame이 0이면 화재가 아님 (온도 조건은 사용하지 않음)
    - blackboard에 저장:
      - fire_detected (bool): 화재 감지 여부
      - fire_level (int): flame 값
      - temp (float): 온도 값
    """
    def __init__(self, name, agent, fire_sensor_topic="/fire_sensor", 
                 flame_threshold=1, temp_threshold=600):
        super().__init__(name, agent, [
            (String, fire_sensor_topic, 'fire_sensor'),
        ])
        # ConditionWithROSTopics는 기본적으로 type="Condition"이므로 명시적으로 설정하지 않음
        self.fire_sensor_topic = fire_sensor_topic
        self.default_flame_threshold = int(flame_threshold)
        self.default_temp_threshold = float(temp_threshold)
        agent.ros_bridge.node.get_logger().info(
            f'UpdateFireState: 구독 시작 - 토픽: {fire_sensor_topic}, '
            f'flame_threshold={flame_threshold}, temp_threshold={temp_threshold}'
        )
    
    def _predicate(self, agent, blackboard) -> bool:
        """센서 데이터를 파싱하여 blackboard에 저장"""
        cache = self._cache
        if "fire_sensor" in cache:
            try:
                # JSON 파싱
                msg = cache["fire_sensor"]
                data = json.loads(msg.data)
                
                flame = data.get("flame", 0)
                temp = data.get("temp", 0)
                
                # 파라미터 가져오기 (blackboard 우선, 없으면 기본값)
                flame_threshold = blackboard.get('fire_flame_threshold', self.default_flame_threshold)
                temp_threshold = blackboard.get('fire_temp_threshold', self.default_temp_threshold)
                
                # 화재 감지 로직: flame >= flame_threshold (불꽃 감지가 우선)
                # 온도 조건은 제거: temp < temp_threshold는 제거 (온도만으로는 화재 판단 불가)
                fire_detected = (flame >= flame_threshold)
                
                # blackboard에 저장
                blackboard['fire_detected'] = fire_detected
                blackboard['fire_level'] = int(flame)
                blackboard['temp'] = float(temp)
                
                agent.ros_bridge.node.get_logger().debug(
                    f'UpdateFireState: 센서 데이터 업데이트 - flame={flame}, temp={temp}, '
                    f'fire_detected={fire_detected} (기준: flame>={flame_threshold})'
                )
                return True
            except json.JSONDecodeError as e:
                agent.ros_bridge.node.get_logger().error(f'UpdateFireState: JSON 파싱 실패 - {e}')
                return False
            except Exception as e:
                agent.ros_bridge.node.get_logger().error(f'UpdateFireState: 처리 실패 - {e}')
                return False
        else:
            # 센서 데이터가 없으면 blackboard 초기화
            if blackboard.get('fire_detected') is not None:
                agent.ros_bridge.node.get_logger().warning(
                    f'UpdateFireState: 센서 데이터 수신 실패 - 토픽 "{self.fire_sensor_topic}"에서 메시지를 받지 못함'
                )
            blackboard['fire_detected'] = False
            blackboard['fire_level'] = 0
            blackboard['temp'] = 0.0
            return False


class StopRobot(Node):
    """
    로봇을 정지시키는 노드.
    - 토픽: /cmd_vel (geometry_msgs/Twist)
    - Twist(linear.x=0.0, angular.z=0.0) 메시지를 발행하여 로봇 정지
    """
    def __init__(self, name, agent, cmd_vel_topic="/cmd_vel"):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.cmd_vel_topic = cmd_vel_topic
        self.type = "Action"
        
        # cmd_vel 퍼블리셔 생성
        self._publisher = self.ros.node.create_publisher(Twist, cmd_vel_topic, 10)
        agent.ros_bridge.node.get_logger().info(
            f'StopRobot: 초기화 완료 - 토픽: {cmd_vel_topic}'
        )
    
    async def run(self, agent, blackboard):
        """로봇 정지 명령 발행"""
        try:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
            self._publisher.publish(twist)
            self.ros.node.get_logger().info('StopRobot: 로봇 정지 명령 발행')
            
            self.status = Status.SUCCESS
            return self.status
            
        except Exception as e:
            self.ros.node.get_logger().error(f'StopRobot: 정지 명령 발행 실패 - {e}')
            self.status = Status.FAILURE
            return self.status


class IsFireDetected(Node):
    """
    blackboard의 fire_detected 값을 확인하여 화재 감지 여부를 판단하는 노드.
    - blackboard['fire_detected'] 값 확인
    - True면 SUCCESS (화재 감지됨)
    - False 또는 없으면 FAILURE (화재 없음)
    """
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.type = "Condition"
        agent.ros_bridge.node.get_logger().info('IsFireDetected: 초기화 완료')
    
    async def run(self, agent, blackboard):
        """blackboard의 fire_detected 값을 확인"""
        fire_detected = blackboard.get('fire_detected', False)
        
        if fire_detected:
            self.ros.node.get_logger().info('IsFireDetected: 화재 감지됨')
            self.status = Status.SUCCESS
        else:
            self.ros.node.get_logger().debug('IsFireDetected: 화재 없음')
            self.status = Status.FAILURE
        
        return self.status


class PlayLocalAlarm(Node):
    """
    로컬 알람을 재생하는 노드 (구현 보류).
    - 알람 재생 방식이 아직 결정되지 않아 주석 처리된 스켈레톤만 구현됨
    - TODO: 알람 재생 방식 결정 후 구현 필요
    - 예상 토픽: /fire/alarm_cmd (std_msgs/String)
    """
    def __init__(self, name, agent, alarm_topic="/fire/alarm_cmd"):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.alarm_topic = alarm_topic
        self.type = "Action"
        
        # TODO: 알람 재생 방식 결정 후 구현
        # 예시: self._publisher = self.ros.node.create_publisher(String, alarm_topic, 10)
        
        agent.ros_bridge.node.get_logger().warn(
            f'PlayLocalAlarm: 스켈레톤만 구현됨 - 알람 재생 방식 결정 필요 (토픽: {alarm_topic})'
        )
    
    async def run(self, agent, blackboard):
        """알람 재생 (구현 보류)"""
        # TODO: 알람 재생 방식 결정 후 구현
        # 예시:
        # try:
        #     msg = String()
        #     msg.data = "FIRE_ALARM"
        #     self._publisher.publish(msg)
        #     self.status = Status.SUCCESS
        #     return self.status
        # except Exception as e:
        #     self.status = Status.FAILURE
        #     return self.status
        
        self.ros.node.get_logger().warn('PlayLocalAlarm: 아직 구현되지 않음 - SUCCESS 반환')
        self.status = Status.SUCCESS
        return self.status


class IsObstacleDetected(ConditionWithROSTopics):
    """
    depth 장애물 감지 여부를 확인하는 노드.
    - 토픽: XML 파라미터 'obstacle_topic' 또는 기본값 "/depth_obstacles"
    - 메시지: std_msgs/Bool
    - True면 SUCCESS (장애물 감지됨), False면 FAILURE (장애물 없음)
    - 참고: depth_obstacle_detector_node.py가 발행하는 토픽을 구독
    """
    def __init__(self, name, agent, obstacle_topic="/depth_obstacles"):
        super().__init__(name, agent, [
            (Bool, obstacle_topic, 'obstacle'),
        ])
        self.obstacle_topic = obstacle_topic
        agent.ros_bridge.node.get_logger().info(
            f'IsObstacleDetected: 구독 시작 - 토픽: {obstacle_topic}'
        )
    
    def _predicate(self, agent, blackboard) -> bool:
        """장애물 감지 여부 확인"""
        cache = self._cache
        if "obstacle" in cache:
            msg = cache["obstacle"]
            obstacle_detected = msg.data
            
            # blackboard에 저장
            blackboard['obstacle_detected'] = obstacle_detected
            
            agent.ros_bridge.node.get_logger().debug(
                f'IsObstacleDetected: 장애물 감지 여부 = {obstacle_detected}'
            )
            return obstacle_detected
        else:
            # 토픽 데이터가 없으면 장애물 없음으로 처리
            blackboard['obstacle_detected'] = False
            return False


class IsNightMode(ConditionWithROSTopics):
    """
    순찰 모드가 야간 모드인지 확인하는 노드.
    - 토픽: XML 파라미터 'patrol_mode_topic' 또는 기본값 "/patrol_mode"
    - 메시지: std_msgs/String ("NIGHT" 또는 "DAY")
    - target_mode와 일치하면 SUCCESS, 아니면 FAILURE
    - 참고: mode_manager_node.py가 발행하는 토픽을 구독
    """
    def __init__(self, name, agent, patrol_mode_topic="/patrol_mode", target_mode="NIGHT"):
        super().__init__(name, agent, [
            (String, patrol_mode_topic, 'patrol_mode'),
        ])
        self.patrol_mode_topic = patrol_mode_topic
        self.target_mode = target_mode.upper()  # 대문자로 변환
        agent.ros_bridge.node.get_logger().info(
            f'IsNightMode: 구독 시작 - 토픽: {patrol_mode_topic}, target_mode: {target_mode}'
        )
    
    def _predicate(self, agent, blackboard) -> bool:
        """순찰 모드 확인"""
        cache = self._cache
        if "patrol_mode" in cache:
            msg = cache["patrol_mode"]
            current_mode = msg.data.strip().upper()  # 공백 제거 후 대문자 변환
            
            # target_mode와 일치하는지 확인
            is_target_mode = (current_mode == self.target_mode)
            
            # blackboard에 저장
            blackboard['patrol_mode'] = current_mode
            blackboard['is_night_mode'] = (current_mode == "NIGHT")
            
            agent.ros_bridge.node.get_logger().debug(
                f'IsNightMode: 현재 모드 = {current_mode}, target_mode = {self.target_mode}, '
                f'일치 여부 = {is_target_mode}'
            )
            return is_target_mode
        else:
            # 토픽 데이터가 없으면 FAILURE
            blackboard['patrol_mode'] = None
            blackboard['is_night_mode'] = False
            return False


class MoveToCoordinate(ActionWithROSAction):
    """
    특정 좌표로 로봇을 이동시키는 노드.
    - 좌표 설정 방법:
      1. XML 파라미터로 설정: x, y, z, yaw (라디안)
      2. blackboard에서 가져오기: 'target_x', 'target_y', 'target_z', 'target_yaw'
    - frame_id: XML 파라미터 'frame_id' 또는 기본값 'map'
    - Nav2의 navigate_to_pose 액션을 사용하여 이동
    - Limo Wiki 참고: https://goofy-pleasure-a84.notion.site/Limo-Wiki-a6aa65b627cb40019a82d469dc5ae69d
    """
    def __init__(self, name, agent, x=0.0, y=0.0, z=0.0, yaw=0.0, frame_id='map', action_name='/navigate_to_pose'):
        super().__init__(name, agent, (NavigateToPose, action_name))
        self.ros = agent.ros_bridge
        self.default_x = float(x)
        self.default_y = float(y)
        self.default_z = float(z)
        self.default_yaw = float(yaw)  # 라디안 단위
        self.default_frame_id = frame_id
        self._goal_sent = False
        
        agent.ros_bridge.node.get_logger().info(
            f'MoveToCoordinate: 초기화 완료 - 기본 좌표: ({x:.2f}, {y:.2f}, {z:.2f}), yaw: {yaw:.2f} rad, frame: {frame_id}'
        )
    
    def _build_goal(self, agent, blackboard):
        """목표 좌표를 설정하여 NavigateToPose Goal 생성"""
        # 좌표 가져오기 (blackboard 우선, 없으면 기본값)
        target_x = blackboard.get('target_x', self.default_x)
        target_y = blackboard.get('target_y', self.default_y)
        target_z = blackboard.get('target_z', self.default_z)
        target_yaw = blackboard.get('target_yaw', self.default_yaw)
        frame_id = blackboard.get('target_frame_id', self.default_frame_id)
        
        # yaw를 쿼터니언으로 변환
        qz = math.sin(target_yaw / 2.0)
        qw = math.cos(target_yaw / 2.0)
        
        # PoseStamped 생성
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame_id
        goal_pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(target_x)
        goal_pose.pose.position.y = float(target_y)
        goal_pose.pose.position.z = float(target_z)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = float(qz)
        goal_pose.pose.orientation.w = float(qw)
        
        # NavigateToPose Goal 생성
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        
        # blackboard에 목표 좌표 저장
        blackboard['current_goal_pose'] = goal_pose
        blackboard['current_goal_x'] = target_x
        blackboard['current_goal_y'] = target_y
        blackboard['current_goal_z'] = target_z
        blackboard['current_goal_yaw'] = target_yaw
        
        self.ros.node.get_logger().info(
            f'MoveToCoordinate: 목표 좌표 설정 - ({target_x:.2f}, {target_y:.2f}, {target_z:.2f}), yaw: {target_yaw:.2f} rad'
        )
        
        return goal
    
    def _interpret_result(self, result, agent, blackboard, status_code=None):
        """네비게이션 결과 해석"""
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            blackboard['nav_result'] = 'succeeded'
            blackboard['nav_status'] = 'success'
            self.ros.node.get_logger().info('MoveToCoordinate: 목표 좌표 도착 성공!')
            return Status.SUCCESS
        elif status_code == GoalStatus.STATUS_CANCELED:
            blackboard['nav_result'] = 'canceled'
            blackboard['nav_status'] = 'canceled'
            self.ros.node.get_logger().warn('MoveToCoordinate: 네비게이션 취소됨')
            return Status.FAILURE
        else:
            blackboard['nav_result'] = 'aborted'
            blackboard['nav_status'] = 'failed'
            self.ros.node.get_logger().error(f'MoveToCoordinate: 네비게이션 실패 (status: {status_code})')
            return Status.FAILURE
    
    def halt(self):
        """노드 중지 시 목표 취소"""
        super().halt()
        self._goal_sent = False


class FollowWaypoints(ActionWithROSAction):
    """
    여러 waypoint를 순차적으로 방문하는 Waypoint 주행 Application 노드.
    - Waypoint 설정 방법:
      1. XML 파라미터로 설정: waypoints="x1,y1,yaw1;x2,y2,yaw2;..." (세미콜론으로 구분, 쉼표로 좌표 구분)
      2. blackboard에서 가져오기: 'waypoints' (리스트 형태: [{'x': x1, 'y': y1, 'z': z1, 'yaw': yaw1}, ...])
    - frame_id: XML 파라미터 'frame_id' 또는 기본값 'map'
    - Nav2의 navigate_to_pose 액션을 사용하여 각 waypoint로 순차 이동
    - 모든 waypoint를 방문하면 SUCCESS 반환
    - 중간에 실패하면 FAILURE 반환
    - Limo Wiki 참고: https://goofy-pleasure-a84.notion.site/ROS2-Navigation-System-975f6cc38d6f417a86bf65f193c89aeb
    """
    def __init__(self, name, agent, waypoints="", frame_id='map', action_name='/navigate_to_pose'):
        super().__init__(name, agent, (NavigateToPose, action_name))
        self.ros = agent.ros_bridge
        self.default_waypoints_str = waypoints
        self.default_frame_id = frame_id
        self._current_waypoint_index = 0
        self._waypoints_list = []
        
        agent.ros_bridge.node.get_logger().info(
            f'FollowWaypoints: 초기화 완료 - waypoints: {waypoints}, frame: {frame_id}'
        )
    
    def _parse_waypoints(self, waypoints_input):
        """
        waypoint 문자열 또는 리스트를 파싱하여 리스트로 변환.
        - 문자열 형식: "x1,y1,yaw1;x2,y2,yaw2;..." (z는 선택적, 기본값 0.0)
        - 리스트 형식: [{'x': x1, 'y': y1, 'z': z1, 'yaw': yaw1}, ...]
        """
        waypoints = []
        
        if isinstance(waypoints_input, str):
            # 문자열 파싱
            if not waypoints_input.strip():
                return waypoints
            
            waypoint_strings = waypoints_input.split(';')
            for wp_str in waypoint_strings:
                wp_str = wp_str.strip()
                if not wp_str:
                    continue
                
                parts = [p.strip() for p in wp_str.split(',')]
                if len(parts) >= 2:
                    try:
                        x = float(parts[0])
                        y = float(parts[1])
                        z = float(parts[2]) if len(parts) > 2 else 0.0
                        yaw = float(parts[3]) if len(parts) > 3 else 0.0
                        waypoints.append({'x': x, 'y': y, 'z': z, 'yaw': yaw})
                    except ValueError as e:
                        self.ros.node.get_logger().error(f'FollowWaypoints: waypoint 파싱 실패 - {wp_str}: {e}')
        
        elif isinstance(waypoints_input, list):
            # 리스트 형식
            for wp in waypoints_input:
                if isinstance(wp, dict):
                    waypoints.append({
                        'x': float(wp.get('x', 0.0)),
                        'y': float(wp.get('y', 0.0)),
                        'z': float(wp.get('z', 0.0)),
                        'yaw': float(wp.get('yaw', 0.0))
                    })
                else:
                    self.ros.node.get_logger().warn(f'FollowWaypoints: 잘못된 waypoint 형식: {wp}')
        
        return waypoints
    
    def _build_goal(self, agent, blackboard):
        """현재 waypoint를 목표로 설정하여 NavigateToPose Goal 생성"""
        # waypoint 리스트 가져오기 (blackboard 우선, 없으면 기본값)
        waypoints_input = blackboard.get('waypoints', self.default_waypoints_str)
        frame_id = blackboard.get('waypoints_frame_id', self.default_frame_id)
        
        # waypoint 리스트 파싱 (최초 1회만)
        if not self._waypoints_list:
            self._waypoints_list = self._parse_waypoints(waypoints_input)
            if not self._waypoints_list:
                self.ros.node.get_logger().error('FollowWaypoints: waypoint가 없습니다.')
                return None
            
            self.ros.node.get_logger().info(
                f'FollowWaypoints: {len(self._waypoints_list)}개의 waypoint 로드됨'
            )
            self._current_waypoint_index = 0
        
        # 모든 waypoint를 방문했는지 확인
        if self._current_waypoint_index >= len(self._waypoints_list):
            self.ros.node.get_logger().info('FollowWaypoints: 모든 waypoint 방문 완료!')
            return None
        
        # 현재 waypoint 가져오기
        current_wp = self._waypoints_list[self._current_waypoint_index]
        target_x = current_wp['x']
        target_y = current_wp['y']
        target_z = current_wp['z']
        target_yaw = current_wp['yaw']
        
        # yaw를 쿼터니언으로 변환
        qz = math.sin(target_yaw / 2.0)
        qw = math.cos(target_yaw / 2.0)
        
        # PoseStamped 생성
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame_id
        goal_pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(target_x)
        goal_pose.pose.position.y = float(target_y)
        goal_pose.pose.position.z = float(target_z)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = float(qz)
        goal_pose.pose.orientation.w = float(qw)
        
        # NavigateToPose Goal 생성
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        
        # blackboard에 현재 waypoint 정보 저장
        blackboard['current_waypoint_index'] = self._current_waypoint_index
        blackboard['total_waypoints'] = len(self._waypoints_list)
        blackboard['current_goal_pose'] = goal_pose
        blackboard['current_goal_x'] = target_x
        blackboard['current_goal_y'] = target_y
        blackboard['current_goal_z'] = target_z
        blackboard['current_goal_yaw'] = target_yaw
        
        self.ros.node.get_logger().info(
            f'FollowWaypoints: Waypoint {self._current_waypoint_index + 1}/{len(self._waypoints_list)} '
            f'목표 좌표 설정 - ({target_x:.2f}, {target_y:.2f}, {target_z:.2f}), yaw: {target_yaw:.2f} rad'
        )
        
        return goal
    
    def _interpret_result(self, result, agent, blackboard, status_code=None):
        """네비게이션 결과 해석"""
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            # 현재 waypoint 도착 성공
            self.ros.node.get_logger().info(
                f'FollowWaypoints: Waypoint {self._current_waypoint_index + 1}/{len(self._waypoints_list)} 도착 성공!'
            )
            
            # 다음 waypoint로 이동
            self._current_waypoint_index += 1
            
            # 모든 waypoint를 방문했는지 확인
            if self._current_waypoint_index >= len(self._waypoints_list):
                blackboard['nav_result'] = 'succeeded'
                blackboard['nav_status'] = 'success'
                blackboard['waypoints_completed'] = True
                self.ros.node.get_logger().info('FollowWaypoints: 모든 waypoint 방문 완료!')
                return Status.SUCCESS
            else:
                # 다음 waypoint로 이동하기 위해 RUNNING 반환 (다음 틱에 다시 실행)
                blackboard['nav_result'] = 'waypoint_reached'
                blackboard['nav_status'] = 'running'
                return Status.RUNNING
                
        elif status_code == GoalStatus.STATUS_CANCELED:
            blackboard['nav_result'] = 'canceled'
            blackboard['nav_status'] = 'canceled'
            self.ros.node.get_logger().warn('FollowWaypoints: 네비게이션 취소됨')
            return Status.FAILURE
        else:
            blackboard['nav_result'] = 'aborted'
            blackboard['nav_status'] = 'failed'
            self.ros.node.get_logger().error(
                f'FollowWaypoints: Waypoint {self._current_waypoint_index + 1} 네비게이션 실패 (status: {status_code})'
            )
            return Status.FAILURE
    
    def halt(self):
        """노드 중지 시 목표 취소 및 상태 리셋"""
        super().halt()
        # waypoint 리스트는 유지 (재시작 시 같은 waypoint 사용)
        # self._current_waypoint_index = 0  # 필요시 주석 해제하여 처음부터 다시 시작


class RegisterPerson(Node):
    """
    감지된 사람을 등록된 사람 데이터베이스에 추가하는 노드.
    - blackboard['detected_person'] 또는 blackboard['detected_{target_class}']에서 감지된 사람 정보 가져오기
    - 사람 이름: XML 파라미터 'person_name' 또는 blackboard['person_name']
    - 등록된 사람 데이터베이스(Chroma)에 임베딩 저장
    
    사용 방법:
    1. XML에서 직접 이름 지정:
       <RegisterPerson target_class="person" person_name="홍길동"/>
    
    2. blackboard에서 이름 가져오기:
       blackboard['person_name'] = "홍길동"
       <RegisterPerson target_class="person"/>
    
    3. ROS 토픽으로 등록 (런타임):
       ros2 topic pub --once /register_person std_msgs/String "data: '홍길동'"
    """
    def __init__(self, name, agent, target_class="person", person_name=None):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.target_class = target_class.lower()
        self.default_person_name = person_name
        self.type = "Action"
        
        # ROS 토픽 구독 (런타임에 사람 등록 요청 받기)
        from std_msgs.msg import String
        self.register_topic = f"/register_person_{self.name}"
        self.ros.node.create_subscription(
            String,
            self.register_topic,
            self._on_register_request,
            10
        )
        self._pending_register_name = None  # 대기 중인 등록 요청
        
        # 등록된 사람용 Chroma 초기화
        if CHROMA_AVAILABLE:
            self._init_registered_person_collection()
        else:
            self.ros.node.get_logger().error(
                'ChromaDB가 설치되지 않았습니다. RegisterPerson 노드를 사용할 수 없습니다.'
            )
        
        agent.ros_bridge.node.get_logger().info(
            f'RegisterPerson: 초기화 완료 - target_class="{self.target_class}", '
            f'토픽: {self.register_topic}'
        )
    
    def _on_register_request(self, msg):
        """ROS 토픽으로 받은 등록 요청 처리"""
        person_name = msg.data.strip()
        if person_name:
            self._pending_register_name = person_name
            self.ros.node.get_logger().info(f'등록 요청 수신: {person_name}')
    
    def _init_registered_person_collection(self):
        """등록된 사람용 Chroma 컬렉션 초기화"""
        global _chroma_clients, _registered_person_collections
        
        try:
            # 새로운 Chroma 클라이언트 생성 방식 (PersistentClient 사용)
            # 등록된 사람용은 별도 클라이언트 사용
            client_key = f"registered_{self.name}"
            if client_key not in _chroma_clients:
                _chroma_clients[client_key] = chromadb.PersistentClient(
                    path=f"./chroma_db_registered_{self.name}"
                )
            
            # 등록된 사람용 컬렉션 생성 또는 가져오기
            collection_name = f"registered_{self.target_class}_{self.name}"
            _registered_person_collections[self.name] = _chroma_clients[client_key].get_or_create_collection(
                name=collection_name,
                metadata={"description": f"Registered {self.target_class} embeddings"}
            )
            
            count = _registered_person_collections[self.name].count()
            self.ros.node.get_logger().info(
                f'등록된 사람 컬렉션 준비 완료: "{collection_name}" (현재 등록된 사람 수: {count})'
            )
        except Exception as e:
            self.ros.node.get_logger().error(f'등록된 사람 컬렉션 초기화 실패: {e}')
    
    def _extract_person_image(self, frame, bbox, use_upper_body=False, focus_top_half=False, focus_bottom_half=True):
        """
        바운딩박스 영역에서 사람 이미지 추출
        - use_upper_body=True: 상체 영역만 추출
        - focus_top_half=True: 상단 50%만 사용
        - focus_bottom_half=True: 하체 50%만 사용 (낮은 각도에서 하체가 더 일관된 특징 제공)
        """
        x1, y1, x2, y2 = map(int, bbox)
        h, w = frame.shape[:2]
        x1 = max(0, min(x1, w))
        y1 = max(0, min(y1, h))
        x2 = max(0, min(x2, w))
        y2 = max(0, min(y2, h))
        
        person_img = frame[y1:y2, x1:x2]
        
        if person_img.size == 0:
            return person_img
        
        img_h = person_img.shape[0]
        
        # 낮은 각도에서는 하체가 더 일관된 특징을 제공 (바지, 신발 등)
        if focus_bottom_half:
            # 하체 50%만 사용 (바지, 신발 등)
            bottom_half_start = int(img_h * 0.5)
            person_img = person_img[bottom_half_start:, :]
        elif focus_top_half:
            # 상단 50%만 사용 (얼굴 + 상체 상단)
            top_half_end = int(img_h * 0.5)
            person_img = person_img[:top_half_end, :]
        elif use_upper_body:
            # 상체 60%만 사용 (얼굴 + 상체)
            upper_body_end = int(img_h * 0.6)
            person_img = person_img[:upper_body_end, :]
        
        return person_img
    
    def _get_image_embedding(self, image, use_enhanced_features=True):
        """
        이미지를 임베딩 벡터로 변환 (개선된 방법)
        - use_enhanced_features=True: 히스토그램, 색상 특징 등 추가
        """
        import numpy as np
        
        if image.size == 0:
            return None
        
        # 이미지 전처리
        # 1. 밝기 정규화 (조명 변화에 강건하게)
        if len(image.shape) == 3:
            lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            l_channel = lab[:, :, 0]
            # CLAHE (Contrast Limited Adaptive Histogram Equalization) 적용
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            l_channel = clahe.apply(l_channel)
            lab[:, :, 0] = l_channel
            image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        
        # 2. 이미지 리사이즈 (더 큰 크기로 - 특징 보존)
        resized = cv2.resize(image, (256, 256))
        
        # 3. RGB로 변환
        if len(resized.shape) == 3 and resized.shape[2] == 3:
            resized_rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        else:
            resized_rgb = resized
        
        # 기본 임베딩: 이미지 픽셀 값
        pixel_features = resized_rgb.flatten().astype(np.float32)
        
        if use_enhanced_features:
            # 추가 특징 추출
            features_list = [pixel_features]
            
            # 4. 색상 히스토그램 특징 (옷 색상 등)
            hist_b = cv2.calcHist([resized], [0], None, [32], [0, 256])
            hist_g = cv2.calcHist([resized], [1], None, [32], [0, 256])
            hist_r = cv2.calcHist([resized], [2], None, [32], [0, 256])
            hist_features = np.concatenate([hist_b.flatten(), hist_g.flatten(), hist_r.flatten()]).astype(np.float32)
            features_list.append(hist_features)
            
            # 5. 텍스처 특징 (LBP - Local Binary Pattern)
            gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY) if len(resized.shape) == 3 else resized
            # 간단한 LBP 구현
            lbp = self._simple_lbp(gray)
            lbp_hist = cv2.calcHist([lbp], [0], None, [256], [0, 256]).flatten().astype(np.float32)
            features_list.append(lbp_hist)
            
            # 6. 하체 영역 강조 (하단 40% 영역의 특징 가중치 증가 - 바지, 신발 등)
            bottom_region = resized_rgb[int(resized_rgb.shape[0] * 0.6):, :]
            bottom_features = bottom_region.flatten().astype(np.float32)
            features_list.append(bottom_features * 1.5)  # 가중치 증가
            
            # 모든 특징 결합
            embedding = np.concatenate(features_list).astype(np.float32)
        else:
            embedding = pixel_features
        
        # L2 정규화
        embedding = embedding / (np.linalg.norm(embedding) + 1e-8)
        
        return embedding.tolist()
    
    def _simple_lbp(self, image):
        """간단한 LBP (Local Binary Pattern) 구현"""
        import numpy as np
        h, w = image.shape
        lbp = np.zeros_like(image)
        
        for i in range(1, h-1):
            for j in range(1, w-1):
                center = image[i, j]
                code = 0
                code |= (image[i-1, j-1] >= center) << 7
                code |= (image[i-1, j] >= center) << 6
                code |= (image[i-1, j+1] >= center) << 5
                code |= (image[i, j+1] >= center) << 4
                code |= (image[i+1, j+1] >= center) << 3
                code |= (image[i+1, j] >= center) << 2
                code |= (image[i+1, j-1] >= center) << 1
                code |= (image[i, j-1] >= center) << 0
                lbp[i, j] = code
        
        return lbp
    
    async def run(self, agent, blackboard):
        import time
        global _registered_person_collections
        
        # 등록된 사람 컬렉션이 없으면 실패
        if self.name not in _registered_person_collections:
            self.ros.node.get_logger().error('등록된 사람 컬렉션이 초기화되지 않았습니다.')
            self.status = Status.FAILURE
            return self.status
        
        # 감지된 사람 정보 가져오기
        detected_person = blackboard.get(f'detected_{self.target_class}')
        if detected_person is None:
            detected_person = blackboard.get('detected_person')
        
        if detected_person is None:
            self.ros.node.get_logger().warn('등록할 사람 정보가 없습니다.')
            self.status = Status.FAILURE
            return self.status
        
        # 사람 이름 가져오기 (우선순위: ROS 토픽 > blackboard > XML 파라미터 > 기본값)
        person_name = None
        
        # 1. ROS 토픽으로 받은 등록 요청 확인
        if self._pending_register_name:
            person_name = self._pending_register_name
            self._pending_register_name = None  # 처리 후 초기화
            self.ros.node.get_logger().info(f'ROS 토픽에서 받은 이름 사용: {person_name}')
        
        # 2. blackboard에서 가져오기
        if not person_name:
            person_name = blackboard.get('person_name')
        
        # 3. XML 파라미터 사용
        if not person_name:
            person_name = self.default_person_name
        
        # 4. 기본 이름 생성
        if not person_name:
            person_name = f"Person_{int(time.time())}"
        
        # 현재 프레임 가져오기
        current_frame = blackboard.get('current_frame')
        if current_frame is None:
            self.ros.node.get_logger().warn('현재 프레임이 없습니다.')
            self.status = Status.FAILURE
            return self.status
        
        # 바운딩박스 가져오기
        bbox = detected_person.get('bbox')
        if not bbox:
            self.ros.node.get_logger().warn('바운딩박스 정보가 없습니다.')
            self.status = Status.FAILURE
            return self.status
        
        # 사람 이미지 추출 및 임베딩 생성 (낮은 각도에 맞게 하체 50%만 사용 - 바지, 신발 등)
        person_img = self._extract_person_image(current_frame, bbox, focus_bottom_half=True)
        if person_img.size == 0:
            self.ros.node.get_logger().warn('사람 이미지 추출 실패')
            self.status = Status.FAILURE
            return self.status
        
        embedding = self._get_image_embedding(person_img, use_enhanced_features=True)
        if embedding is None:
            self.ros.node.get_logger().warn('임베딩 생성 실패')
            self.status = Status.FAILURE
            return self.status
        
        # 등록된 사람 데이터베이스에 추가
        try:
            collection = _registered_person_collections[self.name]
            current_time = time.time()
            
            # 고유 ID 생성
            person_id = f"{person_name}_{int(current_time * 1000)}"
            
            # 중복 확인 (같은 이름이 이미 있으면 업데이트)
            existing = collection.get(ids=[person_id])
            if existing['ids']:
                # 기존 항목 업데이트
                collection.update(
                    ids=[person_id],
                    embeddings=[embedding],
                    metadatas=[{
                        "name": person_name,
                        "timestamp": str(current_time),
                        "bbox": str(bbox),
                        "confidence": str(detected_person.get('confidence', 0.0))
                    }]
                )
                self.ros.node.get_logger().info(f'사람 정보 업데이트: {person_name} (ID: {person_id})')
            else:
                # 새로 추가
                collection.add(
                    embeddings=[embedding],
                    ids=[person_id],
                    metadatas=[{
                        "name": person_name,
                        "timestamp": str(current_time),
                        "bbox": str(bbox),
                        "confidence": str(detected_person.get('confidence', 0.0))
                    }]
                )
                self.ros.node.get_logger().info(f'새 사람 등록 완료: {person_name} (ID: {person_id})')
            
            # 등록된 사람 수 확인
            count = collection.count()
            self.ros.node.get_logger().info(f'현재 등록된 사람 수: {count}')
            
            # blackboard에 저장
            blackboard['registered_person_name'] = person_name
            blackboard['registered_person_id'] = person_id
            
            self.status = Status.SUCCESS
            return self.status
            
        except Exception as e:
            self.ros.node.get_logger().error(f'사람 등록 실패: {e}')
            self.status = Status.FAILURE
            return self.status


# ReadFireSensorFromSerial 노드의 시리얼 포트 인스턴스 관리
_read_fire_sensor_serial_ports = {}  # {node_name: serial.Serial}

class ReadFireSensorFromSerial(Node):
    """
    아두이노 시리얼 포트에서 화재 센서 데이터를 직접 읽어서 blackboard에 저장하는 노드.
    - 시리얼 포트: XML 파라미터 'serial_port' (기본값: "/dev/ttyACM0")
    - 보드레이트: XML 파라미터 'baudrate' (기본값: 9600)
    - 데이터 형식: "Flame: XXX | Temp: YYY.YY" 또는 JSON
    - blackboard에 저장:
      - fire_sensor_raw (str): 원본 문자열 데이터
      - fire_flame_value (int): Flame 값
      - fire_temp_value (float): Temp 값
      - fire_sensor_parsed (dict): 파싱된 JSON 데이터 (있는 경우)
    - 항상 SUCCESS 반환 (데이터가 없으면 기존 blackboard 값 유지)
    """
    def __init__(self, name, agent, serial_port="/dev/ttyACM0", baudrate=9600):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.serial_port = serial_port
        self.baudrate = int(baudrate)
        self.type = "Action"
        
        # 시리얼 포트 초기화
        try:
            import serial
            import time
            self._serial_module = serial
            self._time_module = time
            
            if self.name not in _read_fire_sensor_serial_ports:
                ser = serial.Serial(self.serial_port, self.baudrate, timeout=1.0)
                time.sleep(2)  # 아두이노 리셋 대기
                _read_fire_sensor_serial_ports[self.name] = ser
                agent.ros_bridge.node.get_logger().info(
                    f'ReadFireSensorFromSerial: 시리얼 포트 초기화 완료 - '
                    f'포트: {serial_port}, 보드레이트: {baudrate}'
                )
            else:
                agent.ros_bridge.node.get_logger().info(
                    f'ReadFireSensorFromSerial: 기존 시리얼 포트 사용 - {serial_port}'
                )
        except ImportError:
            agent.ros_bridge.node.get_logger().error(
                'ReadFireSensorFromSerial: pyserial 모듈이 없습니다. pip install pyserial 실행 필요'
            )
            self._serial_module = None
        except Exception as e:
            agent.ros_bridge.node.get_logger().error(
                f'ReadFireSensorFromSerial: 시리얼 포트 초기화 실패 - {e}'
            )
            self._serial_module = None
    
    async def run(self, agent, blackboard):
        """시리얼 포트에서 데이터 읽기"""
        if self._serial_module is None:
            self.status = Status.FAILURE
            return self.status
        
        if self.name not in _read_fire_sensor_serial_ports:
            self.ros.node.get_logger().error(
                'ReadFireSensorFromSerial: 시리얼 포트가 초기화되지 않았습니다'
            )
            self.status = Status.FAILURE
            return self.status
        
        try:
            ser = _read_fire_sensor_serial_ports[self.name]
            
            if ser.in_waiting > 0:
                line_bytes = ser.readline()
                if line_bytes and len(line_bytes) > 0:
                    line = line_bytes.decode('utf-8', errors='ignore').strip()
                    
                    if line:
                        # 원본 데이터 저장
                        blackboard['fire_sensor_raw'] = line
                        
                        # JSON 형식인지 확인
                        if line.startswith("{"):
                            try:
                                data = json.loads(line)
                                blackboard['fire_sensor_parsed'] = data
                                blackboard['fire_flame_value'] = data.get("flame", 0)
                                blackboard['fire_temp_value'] = data.get("temp", 0.0)
                                
                                self.ros.node.get_logger().debug(
                                    f'ReadFireSensorFromSerial: JSON 데이터 읽음 - {data}'
                                )
                            except json.JSONDecodeError as e:
                                self.ros.node.get_logger().warn(
                                    f'ReadFireSensorFromSerial: JSON 파싱 실패 - {e}'
                                )
                        
                        # "Flame: XXX | Temp: YYY.YY" 형식 파싱
                        elif "Flame:" in line and "Temp:" in line:
                            try:
                                # 합쳐진 줄 처리
                                if line.count("Flame:") > 1:
                                    last_flame_idx = line.rfind("Flame:")
                                    line = line[last_flame_idx:].strip()
                                
                                parts = line.split("|")
                                if len(parts) == 2:
                                    flame_part = parts[0].strip()
                                    temp_part = parts[1].strip()
                                    
                                    flame_value = int(flame_part.split(":")[1].strip())
                                    temp_value = float(temp_part.split(":")[1].strip())
                                    
                                    blackboard['fire_flame_value'] = flame_value
                                    blackboard['fire_temp_value'] = temp_value
                                    
                                    self.ros.node.get_logger().debug(
                                        f'ReadFireSensorFromSerial: 데이터 읽음 - '
                                        f'Flame: {flame_value}, Temp: {temp_value}'
                                    )
                            except (ValueError, IndexError) as e:
                                self.ros.node.get_logger().warn(
                                    f'ReadFireSensorFromSerial: 파싱 실패 - {e}, line: {line[:50]}'
                                )
            
            self.status = Status.SUCCESS
            return self.status
            
        except Exception as e:
            self.ros.node.get_logger().error(
                f'ReadFireSensorFromSerial: 읽기 오류 - {e}', exc_info=True
            )
            self.status = Status.FAILURE
            return self.status


class PublishFireSensor(Node):
    """
    blackboard의 화재 센서 데이터를 /fire_sensor 토픽으로 발행하는 노드.
    - blackboard에서 읽기:
      - fire_flame_value (int): Flame 값
      - fire_temp_value (float): Temp 값
      - 또는 fire_sensor_parsed (dict): 파싱된 데이터
    - 발행 토픽: XML 파라미터 'fire_sensor_topic' (기본값: "/fire_sensor")
    - 발행 형식: JSON {"flame": X, "temp": Y.Y}
    - 화재 감지 기준: flame < 600이면 flame_detected = 1
    """
    def __init__(self, name, agent, fire_sensor_topic="/fire_sensor"):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.fire_sensor_topic = fire_sensor_topic
        self.type = "Action"
        
        # 퍼블리셔 생성
        self._publisher = self.ros.node.create_publisher(String, fire_sensor_topic, 10)
        agent.ros_bridge.node.get_logger().info(
            f'PublishFireSensor: 초기화 완료 - 토픽: {fire_sensor_topic}'
        )
    
    async def run(self, agent, blackboard):
        """blackboard 데이터를 토픽으로 발행"""
        try:
            # 파싱된 데이터가 있으면 사용
            if 'fire_sensor_parsed' in blackboard:
                data = blackboard['fire_sensor_parsed']
                flame = data.get("flame", 0)
                temp = data.get("temp", 0.0)
            else:
                # 개별 값 가져오기
                flame = blackboard.get('fire_flame_value', 0)
                temp = blackboard.get('fire_temp_value', 0.0)
            
            # flame 값으로 화재 감지 여부 계산 (flame < 600이면 화재)
            flame_detected = 1 if flame < 600 else 0
            
            # JSON 형식으로 변환
            msg_dict = {
                "flame": flame_detected,
                "temp": float(temp)
            }
            
            msg = String()
            msg.data = json.dumps(msg_dict)
            
            self._publisher.publish(msg)
            self.ros.node.get_logger().debug(
                f'PublishFireSensor: 데이터 발행 - {msg.data}'
            )
            
            self.status = Status.SUCCESS
            return self.status
            
        except Exception as e:
            self.ros.node.get_logger().error(
                f'PublishFireSensor: 발행 실패 - {e}', exc_info=True
            )
            self.status = Status.FAILURE
            return self.status


class IsLidarObstacleDetected(ConditionWithROSTopics):
    """
    LiDAR 기반 장애물 감지 여부를 확인하는 노드.
    - 토픽: "/patrol/obstacle_detected" (원본 object_perception_lidar_node.py와 동일)
    - 메시지: std_msgs/Bool
    - True면 SUCCESS (장애물 감지됨), False면 FAILURE (장애물 없음)
    - 참고: object_perception_lidar_node.py가 발행하는 토픽을 구독
    """
    def __init__(self, name, agent):
        # 원본 object_perception_lidar_node.py의 기본값 사용
        obstacle_topic = "/patrol/obstacle_detected"
        super().__init__(name, agent, [
            (Bool, obstacle_topic, 'lidar_obstacle'),
        ])
        self.obstacle_topic = obstacle_topic
        agent.ros_bridge.node.get_logger().info(
            f'IsLidarObstacleDetected: 구독 시작 - 토픽: {obstacle_topic}'
        )
    
    def _predicate(self, agent, blackboard) -> bool:
        """LiDAR 장애물 감지 여부 확인"""
        cache = self._cache
        if "lidar_obstacle" in cache:
            msg = cache["lidar_obstacle"]
            obstacle_detected = msg.data
            
            # blackboard에 저장
            blackboard['lidar_obstacle_detected'] = obstacle_detected
            
            agent.ros_bridge.node.get_logger().debug(
                f'IsLidarObstacleDetected: LiDAR 장애물 감지 여부 = {obstacle_detected}'
            )
            return obstacle_detected
        else:
            # 토픽 데이터가 없으면 장애물 없음으로 처리
            blackboard['lidar_obstacle_detected'] = False
            return False


class ManagePatrolMode(Node):
    """
    순찰 모드를 설정하거나 카메라 이미지 기반으로 자동으로 모드를 결정하는 노드.
    - 원본 mode_manager_node.py의 기본값 사용:
      - mode_source: 'auto' (자동 모드) 또는 'manual' (수동 모드)
      - initial_mode: 'NIGHT' (수동 모드일 때 초기 모드)
      - brightness_topic: '/camera/image_raw'
      - brightness_threshold: 50.0
      - patrol_mode_topic: '/patrol_mode'
    - 모드 설정 방법:
      1. mode_source='manual'일 때: initial_mode 사용 (기본값 'NIGHT')
      2. mode_source='auto'일 때: 카메라 이미지의 밝기를 기반으로 자동 결정
    - 메시지: std_msgs/String ("NIGHT" 또는 "DAY")
    """
    def __init__(self, name, agent, mode_source='auto'):
        super().__init__(name)
        self.ros = agent.ros_bridge
        
        # 원본 mode_manager_node.py의 기본값 사용
        self.mode_source = mode_source  # 'auto' or 'manual'
        self.initial_mode = 'NIGHT'  # mode_manager_node.py의 기본값
        self.brightness_topic = '/camera/color/image_raw'  # mode_manager_node.py의 기본값
        self.brightness_threshold = 50.0  # mode_manager_node.py의 기본값
        self.patrol_mode_topic = '/patrol_mode'  # mode_manager_node.py의 하드코딩 값
        
        self.type = "Action"
        
        # 모드 퍼블리셔 생성
        self._publisher = self.ros.node.create_publisher(String, self.patrol_mode_topic, 10)
        
        # 자동 모드일 경우 카메라 이미지 구독
        self._bridge = None
        self._current_mode = None
        self.mode = self.initial_mode  # 현재 모드 (원본과 동일하게)
        
        if self.mode_source == 'auto':
            try:
                from cv_bridge import CvBridge
                import cv2
                import numpy as np
                self._bridge = CvBridge()
                self._cv2 = cv2
                self._np = np
                
                self._image_sub = self.ros.node.create_subscription(
                    Image,
                    self.brightness_topic,
                    self._image_callback,
                    10
                )
                agent.ros_bridge.node.get_logger().info(
                    f'ManagePatrolMode: 자동 모드 활성화 - 카메라 토픽: {self.brightness_topic}, '
                    f'밝기 임계값: {self.brightness_threshold}'
                )
            except ImportError:
                agent.ros_bridge.node.get_logger().error(
                    'ManagePatrolMode: cv_bridge 또는 opencv가 없습니다. 자동 모드를 사용할 수 없습니다.'
                )
                self.mode_source = 'manual'
        
        agent.ros_bridge.node.get_logger().info(
            f'ManagePatrolMode: 초기화 완료 - mode_source: {self.mode_source}, '
            f'initial_mode: {self.initial_mode}, 토픽: {self.patrol_mode_topic}'
        )
    
    def _image_callback(self, msg: Image):
        """카메라 이미지 콜백 - 밝기 계산하여 모드 결정 (원본 mode_manager_node.py와 동일 로직)"""
        if self.mode_source != 'auto' or self._bridge is None:
            return
        
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = self._cv2.cvtColor(img, self._cv2.COLOR_BGR2GRAY)
            mean_brightness = float(self._np.mean(gray))
            
            prev_mode = self.mode
            # 원본 mode_manager_node.py와 동일한 로직
            if mean_brightness < self.brightness_threshold:
                self.mode = 'NIGHT'
            else:
                self.mode = 'DAY'
            
            # 원본과 동일한 로그 형식
            if self.mode != prev_mode:
                self.ros.node.get_logger().info(
                    f'Mode changed: {prev_mode} -> {self.mode} (brightness={mean_brightness:.1f})'
                )
                # 모드 발행
                self._publish_mode(self.mode)
            else:
                # 밝은 상황인데도 NIGHT로 인식되는 문제 디버깅을 위해 주기적으로 로그 출력
                self.ros.node.get_logger().info(
                    f'ManagePatrolMode: 현재 모드={self.mode}, 밝기={mean_brightness:.1f}, '
                    f'임계값={self.brightness_threshold} (밝기 < 임계값이면 NIGHT)'
                )
        except Exception as e:
            self.ros.node.get_logger().warn(f'ManagePatrolMode: 이미지 처리 실패 - {e}')
    
    def _publish_mode(self, mode: str):
        """모드 발행"""
        try:
            msg = String()
            msg.data = mode.upper()
            self._publisher.publish(msg)
            self.ros.node.get_logger().debug(f'ManagePatrolMode: 모드 발행 - {mode}')
        except Exception as e:
            self.ros.node.get_logger().error(f'ManagePatrolMode: 모드 발행 실패 - {e}')
    
    async def run(self, agent, blackboard):
        """모드 설정 또는 자동 모드 실행 (원본 mode_manager_node.py의 timer_cb와 유사)"""
        # 원본 mode_manager_node.py의 로직: timer_cb에서 self.mode를 그대로 발행
        # 자동 모드일 경우 이미지 콜백에서 self.mode가 업데이트됨
        # 수동 모드일 경우 self.mode는 initial_mode('NIGHT')를 유지
        
        # 자동 모드인데 이미지가 아직 오지 않았을 때 경고
        if self.mode_source == 'auto' and self._bridge is None:
            self.ros.node.get_logger().warn(
                'ManagePatrolMode: 자동 모드이지만 cv_bridge 초기화 실패. 수동 모드로 전환됨.'
            )
        
        # 모드 발행 (원본과 동일하게)
        mode_upper = self.mode.upper()
        if mode_upper not in ['NIGHT', 'DAY']:
            self.ros.node.get_logger().warn(
                f'ManagePatrolMode: 잘못된 모드 값 - {self.mode} (NIGHT 또는 DAY만 가능)'
            )
            self.status = Status.FAILURE
            return self.status
        
        self._publish_mode(mode_upper)
        
        # blackboard에 저장
        blackboard['patrol_mode'] = mode_upper
        blackboard['is_night_mode'] = (mode_upper == "NIGHT")
        
        self.status = Status.SUCCESS
        return self.status

    

