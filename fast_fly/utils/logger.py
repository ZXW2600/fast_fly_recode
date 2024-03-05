import coloredlogs,logging

def getLogger(name):
    logger=logging.getLogger(name)
    coloredlogs.install(level='DEBUG', logger=logger, fmt='[%(name)15s] [%(levelname)8s] %(asctime)s: %(message)s',
                        datefmt="%H:%M:%S")
    return logger