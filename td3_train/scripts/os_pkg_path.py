import os

# 현재 파일이 위치한 디렉토리의 절대 경로를 가져옵니다.
pkg_path = os.path.abspath(os.path.dirname(__file__))
print("패키지 경로:", pkg_path)

# 한 단계 위의 디렉토리 경로 (cd .. 와 동일)
parent_path = os.path.dirname(pkg_path)
print("부모 디렉토리 경로:", parent_path)
