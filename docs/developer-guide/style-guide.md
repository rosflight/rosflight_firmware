# Style Guide

Any contributions to the firmware should adhere to the following style guidelines  

## White space and Line Endings

Please try not to commit anything that only changes white space or line endings. To check if that's going to happen, run `git diff --check` before you stage your files. Git will warn you about obnoxious changes. Please fix them.

## Code Style
The following ROSflight code style is based on the [ROS C++ style guide](http://wiki.ros.org/CppStyleGuide).
ROSflight uses the C++11 standard.

### Indentation

Indentation should be **2 spaces** (no tabs). Case statements in switch blocks should not be indented, e.g.

    switch (variable)
    {
    case 1:
      // do something
      break;
    default:
      break;
    }

### Braces

Braces should be placed on the next line, e.g.

``` C++
if (i > 2)
{
  // do stuff
}
else
{
  // do something else
}
```

For a conditional with only one statement, the braces can be omitted but the statement should be indented:

``` C++
if (i > 2)
  x = 3;
```

### Spaces

There should be a space between `if`, `for`, or `while` and the condition, e.g. `while (true)`, not `while(true)`.

### Naming Conventions

* Class names should be capitalized with no spaces (i.e. `StateManager`).
* Member variables should contain a post-pended underscore (i.e. `data_`).
* Member functions should be all lower case with underscores (i.e. `set_error()`).
* Integer types should be defined using the `cstdint` convention (i.e. `uint8_t`, `int64_t`, `float` ...).
* Boolean values should be assigned `true` or `false`, not `0` and `1`.

### Function Arguments
Primitive data types (`int`, `float`, etc.) should always be passed by value. Other types (e.g. classes) should be passed by reference and should maintain proper const-correctness. Arguments that are modified by the function should be passed by pointer instead of reference, to make the fact that the argument will be changed clearer in the calling code. For example:

``` C++
void do_something(float dt, const MyClass& data, int* output);
```

This function would be called as

``` C++
float dt = 0.01f;
MyClass my_class;
int value;

do_something(dt, my_class, &value);
```

This makes it clear the `value` is modified by the function call.

### Classes

All modules should be defined as a self-contained class.  All member variables should be declared as "private," named with a post-pended underscore  and accessed through inline accessor functions.  All accessible data should be encapsulated in a struct.  For example, here is a snippet from the `Sensors` module in the firmware:

``` C++
class Sensors
{
public:
  struct Data
  {
    vector_t accel = {0, 0, 0};
    vector_t gyro = {0, 0, 0};
    float imu_temperature = 0;
    uint64_t imu_time = 0;

    float diff_pressure_velocity = 0;
    float diff_pressure = 0;
    float diff_pressure_temp = 0;
    bool diff_pressure_valid = false;

    float baro_altitude = 0;
    float baro_pressure = 0;
    float baro_temperature = 0;
    bool baro_valid = false;

    float sonar_range = 0;
    bool sonar_range_valid = false;

    vector_t mag = {0, 0, 0};

    bool baro_present = false;
    bool mag_present = false;
    bool sonar_present = false;
    bool diff_pressure_present = false;
  };

  Sensors(ROSflight& rosflight);

  inline const Data& data() const { return data_; }

private:
  Data data_;
}
```

Note that `data_` is a private member variable, but the `Data` struct is declared publicly and `data_` is accessed through in `inline const` accessor to prevent another module changing `data_`.

### Enums

Enums should be declared using the following style:
``` C++
enum ArmedState
{
  ARMED_STATE_INIT,
  ARMED_STATE_DISARMED,
  ARMED_STATE_ARMED
};
```

The name of the enum should be in CamelCase, and the names of its members should be ALL_CAPS. Where practical, have the members of the enum begin with the name of the enum.

### Structs

Structs should be declared using the following style:
``` C++
struct SomeValue
{
  int v1;
  int v2;
};
```
Struct type names should be in CamelCase.

### Globals

The use of global variables should be limited to when absolutely necessary (such as linking to interrupt routines or hardware peripherals).  This should only occur in board support layers and not in the core ROSflight libary code.

### Include Order

Include files at the top of your file in the following order:

1. Standard library (e.g. `<cstdint>`)
2. Files from external libraries included in the project (e.g. `<breezystm32/breezystm32.h>`, `<mavlink/v1.0/common/mavlink.h>`)
3. Other header files from this project (e.g. `"rosflight.h"`)
4. The header file for this specific source file

Group the includes according to the above list with an empty line between each group (for external libraries, you may subdivide group 2 into a group for each library). The first two groups should use angle brackets (`<>`), and the last two groups should use quotation marks (`""`). Files from external libraries should be namespaced by the library name (e.g. `<breezystm32/breezystm32.h>`, not `<breezystm32.h>`).

Alphabetize the files within each group . Don't change the include order to fix build errors; if you have to do that it means you aren't including a file somewhere that you should. Please fix it by including all the right files.

For example, in `sensors.c` I might have:
``` C++
#include <cstdbool>
#include <cstdint>

#include <breezystm32/breezystm32.h>
#include <breezystm32/drv_mpu6050.h>

#include "param.h"

#include "sensors.h"
```

### Namespacing

All modules in the firmware should be encapsulated in the `rosflight_firmware` namepace.  This prevents name-clashing in SIL compilation.
