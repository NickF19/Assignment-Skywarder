# Skyward Software Development Assignment (2023-09)

Your task is to develop a simple program that mimics the main task of the flight software, detecting liftoff, apogee and landing events.

The structure is simple, the provided `grader.c` contains the actual main. It parses a csv file to read flight data, and calls first an `init()` funtion and then an `update()` function.

What you need to do is implement the two functions:

```c
// This method will be called once to initialize your code.
void init() {
    // Your code here!
}

// This method will be called for every data point, simulating a full flight.
void update(float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y,
            float gyro_z, float baro) {
    // Your code here!
}
```

Then, inside your `update` function you will need to detect the appropriate events, and call the relative function at the correct time (`liftoff()`, `apogee()`, `landed()`).

You can look at the provided `example.c` for a very simple algorithm, that in practice would never work. In your implementation you should at least use the provided inputs to detect these events.
