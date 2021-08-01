import time


class TimedLooper:
    """A class to easily control how timed loops are run.

    Usage::

        looper = TimedLooper(dt=0.01)
        while looper:
            ... do stuff ...
            if need to stop:
                looper.stop()
                #or just call break

    Note that if dt is too small (or rate is too large), the timing will not
    be accurate due to the system scheduler resolution.

    If the code within the loop takes more than dt seconds to run, then a
    warning may be printed.  To turn this off, set ``warnings=0`` in the
    constructor.  By default, this will print a warning on the first overrun,
    and every ``warning_frequency`` overruns thereafter.

    Args:
        dt (float, optional): the desired time between loops (in seconds)
        rate (float, optional): the number of times per second to run this
            loop (in Hz).  dt = 1/rate.  One of dt or rate must be specified.
        warning_frequency (int, optional): if the elapsed time between calls
            exceeds dt, a warning message will be printed at this frequency.
            Set this to 0 to disable warnings.
        name (str, optional): a descriptive name to be used in the warning
            string.

    Warning: DO NOT attempt to save some time and call the TimedLooper()
    constructor as the condition of your while loop!  I.e., do not do this::

        while TimedLooper(dt=0.01):
            ...

    """

    def __init__(self, dt=None, rate=None, warning_frequency="auto", warning_printer="auto", name=None):
        self.dt = dt
        if dt is None:
            if rate is None:
                raise AttributeError("One of dt or rate must be specified")
            self.dt = 1.0 / rate
        if self.dt < 0:
            raise ValueError("dt must be positive")
        if warning_frequency == "auto":
            warning_frequency = int(2.0 / self.dt)
        if warning_printer == "auto":
            warning_printer = print
        self.warning_frequency = warning_frequency
        self.warning_printer = warning_printer
        self.name = name
        self._iters = 0
        self._time_overrun_since_last_warn = 0
        self._iters_of_last_warn = 0
        self._num_overruns_since_last_warn = 0
        self._num_overruns = 0
        self._warn_count = 0
        self._tstart = None
        self._tlast = None
        self._tnext = None
        self._exit = False

    def stop(self):
        self._exit = True

    def __nonzero__(self):
        return self.__bool__()

    def __bool__(self):
        if self._exit:
            return False
        tnow = time.time()
        if self._tlast is None:
            self._tstart = tnow
            self._tnext = tnow + self.dt
        else:
            elapsed_time = tnow - self._tnext
            if elapsed_time > self.dt:
                self._num_overruns += 1
                self._num_overruns_since_last_warn += 1
                self._time_overrun_since_last_warn += elapsed_time - self.dt
                if (
                    self.warning_frequency > 0
                    and self._num_overruns % self.warning_frequency == 0
                ):
                    ave_overrun = (
                        self._time_overrun_since_last_warn
                        / self._num_overruns_since_last_warn
                    )
                    self.warning_printer(
                        "{}: exceeded loop time budget {:.4f}s on {}/{} iters, by {:4f}s on average".format(
                            ("TimedLooper" if self.name is None else self.name),
                            self.dt,
                            self._num_overruns_since_last_warn,
                            self._iters - self._iters_of_last_warn,
                            ave_overrun,
                        )
                    )
                    self._iters_of_last_warn = self._iters
                    self._time_overrun_since_last_warn = 0
                    self._num_overruns_since_last_warn = 0
                    self._warn_count += 1
                self._tnext = tnow
            else:
                self._tnext += self.dt
                assert (
                    self._tnext >= tnow
                ), "Uh... elapsed time is > dt but tnext < tnow: %f, %f, %f" % (
                    elapsed_time,
                    self._tnext,
                    tnow,
                )
        self._iters += 1
        time.sleep(self._tnext - tnow)
        self._tlast = time.time()
        return True

    def time_elapsed(self):
        """Returns the total time elapsed from the start, in seconds"""
        return time.time() - self._tstart if self._tstart is not None else 0

    def iters(self):
        """Returns the total number of iters run"""
        return self._iters

