# Style Guide

Any contributions to the firmware should adhere to the following style guidelines  

## White space and Line Endings

Please try not to commit anything that only changes white space or line endings. To check if that's going to happen, run `git diff --check` before you stage your files. Git will warn you about obnoxious changes. Please fix them.

## Code Style

### Braces

Braces should be placed on the next line, e.g.

    if (i > 2)
    {
      // do stuff
    }
    else
    {
      // do something else
    }

For a conditional with only one statement, the braces can be ommitted but the statement should be indented:

    if (i > 2)
      x = 3;

### Indentation

Indentation should be 2 spaces (no tabs). Case statements in switch blocks should not be indented, e.g.

    switch (variable)
    {
    case 1:
      // do something
      break;
    default:
      break;
    }

### Spaces

There should be a space between `if`, `for`, or `while` and the condition, e.g. `while (true)`, not `while(true)`.

## Include Order

Include files at the top of your file in the following order:

1. Standard library (e.g. `<stdint.h>`)
2. Files from external libraries included in the project (e.g. `<breezystm32/breezystm32.h>`, `<mavlink/v1.0/common/mavlink.h>`)
3. Other header files from this project (e.g. `"param.h"`)
4. The header file for this specific source file

Group the includes according to the following list with an empty line between each group (for external libraries, you may subdivide group 2 into a group for each library). The first two groups should use angle brackets (`<>`), and the last two groups should use quotation marks (`""`). Files from external libraries should be namespaced by the library name (e.g. `<breezystm32/breezystm32.h>`, not `<breezystm32.h>`).

Alphabetize the files within each group . Don't change the include order to fix build errors; if you have to do that it means you aren't including a file somewhere that you should. Please fix it by including the all right files.

For example, in `sensors.c` I might have:

    #include <stdbool.h>
    #include <stdint.h>

    #include <breezystm32/breezystm32.h>
    #include <breezystm32/drv_mpu6050.h>

    #include "param.h"

    #include "sensors.h"

## Enums

Enums should be declared using the following style:

    typedef enum
    {
      ARMED_STATE_INIT,
      ARMED_STATE_DISARMED,
      ARMED_STATE_ARMED
    } armedState_t;

The name of the enum should be in camelCase and end with `_t`, and the names of its members should be ALL_CAPS. Where practical, have the members of the enum begin with the name of the enum.

## Structs

Structs should be declared using the following style:

    typedef struct
    {
      int v1;
      int v2;
    } someValue_t;

Struct typedef names should end with `_t`. For the first part of the name, use camelCase for multi-word names.

## Typedefs

For clarity, all other typedef names should also end with `_t`, e.g. `param_t` and use camelCase for multi-word names.

## Global Variables

Global variable names should start with an underscore. They should be *declared* in the header file to which they are most relevant using the `extern` keyword. They should be *defined* (or instantiated) in the corresponding source file.

For example, a global variable for IMU data would be declared in `sensors.h` using `extern imuData_t _imu_data;`. It would be defined in `sensors.c` using `imuData_t _imu_data;`.

## Static Variables and Methods

Variables that are scoped as global for the functions in that file, but not intended for use in other files, should be declared `static` and should only be declared in that source file, not the header file.

Functions that are only for use within a specific file should also be declared `static` and should not be declared in the header file, but instead the source file.