import logging


_LOG_FORMAT = "[%(levelname)s] %(name)s: %(message)s"


def configure_logging(level=logging.INFO):
    root_logger = logging.getLogger("ROOT")
    if not root_logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter(_LOG_FORMAT))
        root_logger.addHandler(handler)
    root_logger.setLevel(level)
    return root_logger


def get_logger(name: str):
    configure_logging()
    logger_name = "ROOT"
    if name and name != "__main__":
        logger_name = f"{logger_name}.{name}"
    return logging.getLogger(logger_name)
