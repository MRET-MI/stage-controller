import sys
from pathlib import Path

# gui/ をルートとしてパスを通す
sys.path.insert(0, str(Path(__file__).parent))

from ui.app import main

CONFIG_PATH = Path(__file__).parent / "stage_config.json"

if __name__ == "__main__":
    main(config_path=CONFIG_PATH)
