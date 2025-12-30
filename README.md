# ev3svc
This project is an alternative (not so tiny) wrapper for the EV3 driver, written in C++.
It is intented to be used as an alternative API for userspace applications.
This library is almost header-only except for minimal classes that initializes state of this library.
This library can competing with ev3api library, so don't use **motor apis** of ev3api.h, and use this library completely instead.

## How to use
1. Paste everything to `sdk/common/library/ev3svc/` directory. (create it)
1. Add `-I$(EV3RT_ROOT)/sdk/common/library/ev3svc/include` to your `CXXFLAGS` in `Makefile.inc`.
1. Insert a row into `app_common.cfg`:
```
INCLUDE("ev3api.cfg");
INCLUDE("ev3svc.cfg"); <-- Add this line

ATT_MOD("log_output.o");
ATT_MOD("vasyslog.o");
ATT_MOD("t_perror.o");
ATT_MOD("strerror.o");
ATT_MOD("libc.a");
```

## Natural Language
We mainly use English for primary code comments, but some parts might be written in Japanese.
We're working on making Japanese documentation along English ones. Please feel free to contribute if you can help.
We don't limit language for discussions in issues, pull requests, etc., but English or Japanese is preferred.

## License
This project is licenced under MIT License. See the LICENSE file for details.

## Thanks
This library depends on toppers/ev3rt. We really appreciate all the contributors of toppers/ev3rt.