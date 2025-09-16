import carla

# CARLA 서버에 연결
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# 맵 로딩: 확장자 없이 .umap 이름만 사용
world = client.load_world("FMTC")

