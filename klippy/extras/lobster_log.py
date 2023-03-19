import logging, traceback


def log(msg, col='g'):
    (file, line, func, text) = traceback.extract_stack()[-2]  # type: (str, int, str, str)
    logging.info("[%s:%s]: %s {%s}" % (file.split("/")[-1], line, msg, col))


def signpost(msg, col='g'):
    (file, line, func, text) = traceback.extract_stack()[-2]
    logging.info("==================== %s [%s @%s] ===================={%s}" % (msg, file.split("/")[-1], line, col))


def trace(col='gr', depth=-1):
    tb = reversed(traceback.extract_stack()[:-1])

    for (i, (file, line, func, text)) in enumerate(tb):
        file = file.replace("/home/pi/klipper/klippy/", "")
        logging.info("%sFile \"%s\", line %d, in %s {%s}" % ("\t" if i >0 else "", file, line, func, col))
        if i >= depth >= 0:
            return



