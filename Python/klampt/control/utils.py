"""Utilities for using and building controllers.

.. versionadded:: 0.9
"""

import time
from threading import Lock

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



class PromiseTimeout(Exception):
    def __init__(self, promise):
        self.promise = promise

    def __str__(self):
        if self.promise._name is not None:
            return "PromiseTimeout %s timed out" % (self.promise._name,)
        return "PromiseTimeout timed out"


class PromiseError(Exception):
    def __init__(self, promise, error):
        self.promise = promise

    def __str__(self):
        if self.promise._name is not None:
            return "Promise {} encountered exception {}".format(
                self.promise._name, self.promise._error
            )
        return "Promise encountered exception {}".format(self.promise._error)


class Promise:
    """A placeholder for an asynchronous result, inspired by the twisted
    package.  This provides a thread-safe mechanism to pass values between
    threads.

    For receivers
    -------------

    To wait for result using polling::

        if promise:
            val = promise.value()

    or setting a callback function fn(value) with::

        promise.setCallback(fn)

    or blocking until it arrives using:::

        val = promise.wait()

    If you wish to put a timeout on :func:`wait`, use
    ``val = promise.wait(timeout)``.

    :func:`value` will raise a RuntimeError if the value is not available yet.

    ``wait(timeout)`` will raise a :class:`PromiseTimeout` exception if the
    timeout is reached without the result arriving.

    For senders
    -------------

    Make sure to return a Promise

    Use ``callback(value)`` to set the value when it is available.

    If an error occurred before the value can be delivered, call
    :func:`errback`.

    """

    def __init__(self, name=None):
        self._name = name
        self._read = False
        self._value = None
        self._error = None
        self._callback = None
        self._errback = None
        self._lock = Lock()

    def setCallback(self, fn):
        """Called by receiver to trigger a call fn(value) when a value
        arrives. If a non-error result is already set, fn(value) is called
        immediately.
        """
        assert callable(fn), "fn nees to be a callable object"
        with self._lock:  # locking needed in case the callback is set, then the sender sets the value, then _callback is called twice
            if self._callback is not None:
                raise RuntimeError(
                    "Callback already previously set to {}".format(self._callback)
                )
            self._callback = fn
            if self._read:
                self._callback(self._value)

    def setErrback(self, fn):
        """Called by receiver to trigger a call fn(error) when an error
        arrives. If an error result is already set, fn(error) is called
        immediately.
        """
        assert callable(fn), "fn nees to be a callable object"
        with self._lock:  # locking needed in case the errback is set, then the sender sets the error, then _errback is called twice
            self._errback = fn
            if self._read and self._error:
                self._errback(self._error)

    def __nonzero__(self):
        """Returns True if the provider has responded with a value or error."""
        return self.available()

    def __bool__(self):
        """Returns True if the provider has responded with a value or error."""
        return self.available()

    def available(self):
        """Returns True if the provider has responded with a value or error."""
        return self._read

    def error(self):
        """Returns a previously set error result, or None if no error
        occurred."""
        return self._error

    def value(self):
        """Returns the value previously set by the provider.  A RuntimeError is
        raised if no value has been set yet.

        If an error occurred, a PromiseError is raised.
        """
        if self._read:
            if self._error is not None:
                raise PromiseError(self)
            return self._value
        raise RuntimeError("Value is not available")

    def wait(self, timeout=None, resolution=0.01):
        """Blocks until the response is available and then returns the value.

        A timeout can be used via the timeout argument.  At the moment, this
        just uses polling.  For finer control on the polling frequency,
        change the resolution argument.  (Note: a future implementation might
        do away with polling.)
        """
        t0 = time.time()
        while not self.available():
            time.sleep(resolution)
            t = time.time()
            if timeout is not None and t - t0 > timeout:
                raise PromiseTimeout(self)
        if self.error():
            raise RuntimeError(self.error())
        return self.value()

    def callback(self, value):
        """Called by the provider to set the value."""
        assert not callable(
            value
        ), "callback() is meant to be used by the caller to set the promise's value. Do you mean to use setCallback()?"
        if self._read:
            raise RuntimeError("Can't call callback() twice on a Promise object")
        with self._lock:
            self._value = value
            if self._callback is not None:
                self._callback(value)
            self._read = True

    def errback(self, error):
        """Called by the provider to indicate an error."""
        assert not callable(
            error
        ), "errback() is meant to be used by the caller to set the promise's error condition. Do you mean to use setErrback()?"
        if self._read:
            raise RuntimeError(
                "Can't call errback()/callback() twice on a Promise object"
            )
        with self._lock:
            self._error = error
            if self._errback is not None:
                self._errback(error)
            self._read = True
