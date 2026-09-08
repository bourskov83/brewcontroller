# logger_config.py
from loguru import logger
import logging
import sys

# ------------------------
# 1. Loguru setup
# ------------------------
logger.remove()
logger.add(sys.stdout,
           format="<green>{time:YYYY-MM-DD HH:mm:ss}</green> | "
                  "<level>{level: <8}</level> | "
                  "<cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> - "
                  "<level>{message}</level>",
           level="DEBUG",  # default level for your modules
           colorize=True)

#logger.add("logs/my_project.log", rotation="10 MB", retention="7 days", level="DEBUG")

# ------------------------
# 2. Redirect standard logging to loguru
# ------------------------
class InterceptHandler(logging.Handler):
    def emit(self, record):
        # forward logging messages to loguru
        logger_opt = logger.opt(exception=record.exc_info, depth=2)
        level = record.levelname
        if level not in logger._core.levels:  # if unknown, fallback
            level = record.levelno
        logger_opt.log(level, record.getMessage())

logging.basicConfig(handlers=[InterceptHandler()], level=0, force=True)

# ------------------------
# 3. Set pymodbus root logger level to WARNING
# ------------------------
import logging
logging.getLogger("pymodbus").setLevel(logging.INFO)

# ------------------------
# 4. Expose loguru logger
# ------------------------
log = logger
