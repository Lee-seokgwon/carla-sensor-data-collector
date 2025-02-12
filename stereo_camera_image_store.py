import carla
import os
import numpy as np
from datetime import datetime
import time
import cv2
import pygame
import sys

def save_image(image, camera_type):
    save_dir = f'./{camera_type}_images'
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    filename = f'{save_dir}/{timestamp}.png'
    
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = array.reshape((image.height, image.width, 4))
    array = array[:, :, :3]
    
    cv2.imwrite(filename, array)

def try_spawn_vehicle(world, vehicle_bp, spawn_points, clear_existing=True):
    if clear_existing:
        print("기존 액터들을 제거합니다...")
        for actor in world.get_actors().filter('vehicle.*'):
            actor.destroy()
        
    for spawn_point in spawn_points:
        try:
            vehicle = world.spawn_actor(vehicle_bp, spawn_point)
            print(f"차량이 스폰되었습니다. 스폰 위치: {spawn_point}")
            return vehicle
        except:
            continue
    
    raise Exception("사용 가능한 스폰 포인트를 찾을 수 없습니다.")

def main():
    try:
        print("CARLA 서버에 연결 중...")
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()

        # pygame 초기화
        pygame.init()
        screen = pygame.display.set_mode((800, 600))
        pygame.display.set_caption('CARLA Manual Control')
        
        # 녹화 상태 변수
        global recording
        recording = False
        
        # 차량 스폰
        vehicle_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
        spawn_points = world.get_map().get_spawn_points()
        np.random.shuffle(spawn_points)
        vehicle = try_spawn_vehicle(world, vehicle_bp, spawn_points)

        # 카메라 설정
        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '800')
        camera_bp.set_attribute('image_size_y', '600')
        camera_bp.set_attribute('fov', '90')

        # 카메라 생성 및 부착
        left_camera_transform = carla.Transform(carla.Location(x=2.0, y=-0.5, z=1.5))
        right_camera_transform = carla.Transform(carla.Location(x=2.0, y=0.5, z=1.5))
        
        left_camera = world.spawn_actor(camera_bp, left_camera_transform, attach_to=vehicle)
        right_camera = world.spawn_actor(camera_bp, right_camera_transform, attach_to=vehicle)

        # 카메라 콜백 함수
        def left_camera_callback(image):
            if recording:
                save_image(image, 'left')

        def right_camera_callback(image):
            if recording:
                save_image(image, 'right')

        left_camera.listen(left_camera_callback)
        right_camera.listen(right_camera_callback)

        # 초기 설정
        clock = pygame.time.Clock()
        print("\n조작법:")
        print("W: 전진")
        print("S: 후진")
        print("A: 좌회전")
        print("D: 우회전")
        print("SPACE: 브레이크")
        print("R: 녹화 시작/정지")
        print("ESC: 종료\n")

        # 게임 루프
        running = True
        while running:
            # 이벤트 처리
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_r:
                        recording = not recording
                        status = "시작됨" if recording else "정지됨"
                        print(f"녹화가 {status}")

            # 키 입력 처리
            keys = pygame.key.get_pressed()
            
            # 차량 제어
            control = carla.VehicleControl()
            control.throttle = 1.0 if keys[pygame.K_w] else 0.0
            control.brake = 1.0 if keys[pygame.K_SPACE] else 0.0
            control.steer = -0.5 if keys[pygame.K_a] else 0.5 if keys[pygame.K_d] else 0.0
            control.reverse = keys[pygame.K_s]
            vehicle.apply_control(control)

            # 스펙테이터 카메라 업데이트
            spectator = world.get_spectator()
            vehicle_transform = vehicle.get_transform()

            # 카메라 위치 계산
            spectator_transform = carla.Transform()
            spectator_transform.location = vehicle_transform.location + carla.Location(
                x=-8 * np.cos(np.radians(vehicle_transform.rotation.yaw)),
                y=-8 * np.sin(np.radians(vehicle_transform.rotation.yaw)),
                z=3
            )
            
            # 카메라 방향 계산
            direction = vehicle_transform.location - spectator_transform.location
            spectator_transform.rotation = carla.Rotation(
                pitch=-15,
                yaw=np.degrees(np.arctan2(direction.y, direction.x)),
                roll=0
            )
            
            spectator.set_transform(spectator_transform)

            # 화면 업데이트
            pygame.display.flip()
            clock.tick(60)
            world.tick()

    except Exception as e:
        print(f'오류가 발생했습니다: {e}')
        
    finally:
        print('정리 작업을 시작합니다...')
        pygame.quit()
        
        # 액터 정리
        if 'left_camera' in locals():
            left_camera.destroy()
        if 'right_camera' in locals():
            right_camera.destroy()
        if 'vehicle' in locals():
            vehicle.destroy()
        
        print('모든 액터가 제거되었습니다.')

if __name__ == '__main__':
    main()