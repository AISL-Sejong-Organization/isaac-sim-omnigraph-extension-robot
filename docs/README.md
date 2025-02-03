# OmniGraph Extension - [omni.aisl.robot.extension]

## 개요  
이 확장은 **OmniGraph**를 사용하여 **Pick & Place 작업**을 수행하는 기능을 제공합니다.  
사용자는 물체의 크기, 위치 및 방향을 입력하고, 목표 위치 및 방향을 설정하면,  
시스템이 **Grasp(잡기)** 및 **Release(놓기)** 명령을 판단하여 출력합니다.  

## 주요 기능  
- **Pick & Place 자동화**: 주어진 입력값을 기반으로 물체를 이동  
- **Grasp 및 Release 신호 제공**: 물체를 잡을지 여부를 출력  
- **OmniGraph 노드 기반**: NVIDIA Omniverse 환경에서 동작  

## 입력 (Inputs)  
OmniGraph 노드는 다음과 같은 입력값을 받습니다.  

| 입력 변수 | 데이터 타입 | 설명 |  
|----------|-----------|------|  
| `scale` | `double vector3` | 물체의 크기 (x, y, z) |  
| `object_position` | `double vector3` | 물체의 현재 위치 (x, y, z) |  
| `object_orientation` | `double vector4` | 물체의 방향 (쿼터니언: w, x, y, z) |  
| `target_position` | `double vector3` | 목표 위치 (x, y, z) |  
| `target_orientation` | `double vector4` | 목표 방향 (쿼터니언: w, x, y, z) |  

## 출력 (Outputs)  
| 출력 변수 | 데이터 타입 | 설명 |  
|----------|-----------|------|  
| `grasp_command` | `bool` | `true`이면 물체를 잡고, `false`이면 놓기 |  

## 사용법 (Usage)  
1. **OmniGraph에서 `omni.aisl.robot.extension` 노드 추가**  
2. **입력 값 설정**  
   - 물체의 위치 및 크기, 목표 위치 등을 설정  
3. **OmniGraph 실행**  
   - 노드 실행 후 `grasp_command`를 통해 잡기 또는 놓기 여부 확인  
4. **로봇에 적용**  
   - `grasp_command == true`이면 물체를 잡기  
   - `grasp_command == false`이면 물체를 놓기  