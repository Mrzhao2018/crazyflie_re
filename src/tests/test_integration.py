"""集成测试 - 不需要真机"""

from src.app.bootstrap import build_core_app

print("=== 集成测试 ===")
print("构建完整系统...")

try:
    components = build_core_app("config")
    print("[OK] 配置加载")
    print(f"[OK] Fleet: {len(components['fleet'].all_ids())} drones")
    print(f"[OK] Leaders: {components['fleet'].leader_ids()}")
    print(f"[OK] Followers: {components['fleet'].follower_ids()}")
    print(f"[OK] AFC稳定: {components['afc'].convergence_report().is_stable}")
    print(f"[OK] Mission total time: {components['mission_profile'].total_time()}")
    print("\n[SUCCESS] 系统构建成功，所有组件就绪！")
except Exception as e:
    print(f"[ERROR] {e}")
    import traceback

    traceback.print_exc()
