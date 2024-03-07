import coloredlogs,logging
import time
import os

time_now=time.localtime()

LOG_DATA_FMT='%Y_%m_%d_%H_%M_%S'
LOG_FOLDER="./log"
LOG_FILENAME=f"{time.strftime(LOG_DATA_FMT,time_now)}.log"
if not os.path.exists(LOG_FOLDER):
    os.makedirs(LOG_FOLDER)

LOG_PATH=os.path.join(LOG_FOLDER,LOG_FILENAME)    
# Handler for logfile, 50MB per file, store last 5000 files
RFHhdlr = logging.FileHandler(LOG_PATH)
RFHhdlr.setLevel(logging.DEBUG)
RFHhdlr.setFormatter(logging.Formatter('[%(name)15s] [%(levelname)8s] %(asctime)s: %(message)s'))

def getLogger(name):
    logger=logging.getLogger(name)
    logger.addHandler(RFHhdlr)
    coloredlogs.install(level='DEBUG', logger=logger, fmt='[%(name)15s] [%(levelname)8s] %(asctime)s: %(message)s',
                        datefmt="%H:%M:%S")
    return logger

class Logger:
    INFO=logging.INFO
    DEBUG=logging.DEBUG
    ERROR=logging.ERROR
    WARNING=logging.WARNING
    CRITICAL=logging.CRITICAL


    def __init__(self, name):
        self.logger = getLogger(name)

    def info(self, msg):
        self.logger.info(msg)
    
    def debug(self, msg):
        self.logger.debug(msg)

    def error(self, msg):
        self.logger.error(msg)

    def warning(self, msg):
        self.logger.warning(msg)

    def critical(self, msg):
        self.logger.critical(msg)

    def exception(self, msg):
        self.logger.exception(msg)
        raise Exception(msg)
    
    def log(self, level, msg):
        self.logger.log(level, msg)