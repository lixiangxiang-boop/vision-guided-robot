"""
Agent Configuration — 从环境变量或 .env 文件读取
"""
import os
from dataclasses import dataclass, field

# 尝试从 .env 文件加载 (服务器部署时)
try:
    from dotenv import load_dotenv
    _env_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), '.env')
    load_dotenv(_env_path)
except ImportError:
    pass


@dataclass
class AgentConfig:
    api_key: str = field(default_factory=lambda: os.getenv("DEEPSEEK_API_KEY", ""))
    base_url: str = field(default_factory=lambda: os.getenv("DEEPSEEK_BASE_URL", "https://api.deepseek.com/v1"))
    model: str = field(default_factory=lambda: os.getenv("DEEPSEEK_MODEL", "deepseek-chat"))
    temperature: float = 0.7
    max_tokens: int = 2048
    max_rounds: int = 15
    vision_url: str = field(default_factory=lambda: os.getenv("VISION_URL", "http://127.0.0.1:8801"))
    robot_url: str = field(default_factory=lambda: os.getenv("ROBOT_URL", "http://127.0.0.1:8802"))


def get_config() -> AgentConfig:
    cfg = AgentConfig()
    if not cfg.api_key:
        raise RuntimeError(
            "DEEPSEEK_API_KEY 未设置。请在 .env 文件中配置或设置环境变量。\n"
            "  echo 'DEEPSEEK_API_KEY=sk-your-key' > .env\n"
            "或:\n"
            "  export DEEPSEEK_API_KEY=sk-your-key"
        )
    return cfg
