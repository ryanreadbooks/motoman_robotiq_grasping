# coding=utf-8

import types


def get_traceback(traceback: types.TracebackType, max_depth=10):
    """
    获取抛出异常时的调用栈,方便调试用
    """
    traceback_str = ''
    for i in range(max_depth):
        if traceback is not None:
            filename = traceback.tb_frame.f_code.co_filename
            lineno = traceback.tb_lineno
            traceback_str += f'[{i}]: filename = {filename}, lineno = {lineno}\n' 
            traceback = traceback.tb_next
        else:
            break
    return traceback_str


def get_exception_trackback(excep: Exception, max_depth=10):
    return get_traceback(excep.__traceback__, max_depth=max_depth)
