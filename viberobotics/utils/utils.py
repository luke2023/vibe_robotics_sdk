from viberobotics.constants import ASSET_DIR

def get_asset_path(asset_name: str) -> str:
    return (ASSET_DIR / asset_name).as_posix()