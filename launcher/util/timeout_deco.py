import signal
from functools import wraps
import time

def timeout(seconds, default=None):
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            def signal_handler(signum, frame):
                raise TimeoutError("Timed out!")
            # Set up the signal handler for timeout
            signal.signal(signal.SIGALRM, signal_handler)

            # Set the initial alarm for the integer part of seconds
            signal.setitimer(signal.ITIMER_REAL, seconds)

            
            try:
                result = func(*args, **kwargs)
            except TimeoutError:
                return default
            finally:
                signal.alarm(0)
            
            return result
        
        return wrapper
    
    return decorator