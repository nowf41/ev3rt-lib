## flags
### EV3SVC_SKIP_UNSIGNED_CHECK

  If defined, the checks for unsigned parameters (like some parameters that takes power level, time in milliseconds) are skipped.
  
  If you are going to use very large values for those parameters, you can define this flag to skip the checks.

### EV3SVC_TERMINATE_TASK_ON_INVALID_PARAMETER

  If defined, the functions in this library will terminate the task when invalid parameters are given (like power level is 0 or less).
  
  If not defined, the functions will return ERROR_CODE::INVALID_PARAM instead.

<br>

By default, those flags are not defined.