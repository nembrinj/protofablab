# Pipeline.json

This config file describes the various methods for an image editing pipeline.
<br>
Each method has different arguments like `kernel size`, `threshold`, `iterations`, etc. with some constraints. These
constraints are listed here along with a recommended value. This recommendation, however, depends on the image it is
being applied to. Therefore, user discretion is advised and the values only serve as a guide line.

## Structure
The overall structure is a list of methods and their descriptions:
```json
{
    "method 1": { ... }
    "method 2": { ... }
    ...
}
```

### Methods
Each method is either a direct function with arguments, or has different implementations which can be chosen.

A single method with arguments is described like the following:
```json
"method name": {
    "argument 1": {
        "constraint 1": /* ... */,      // e.g. "min": 0
        "constraint 2": /* ... */,      // e.g. "step": 2
        // ...
        "recommended": /* ... */,       // e.g. "recommended": 127
        "description": "...",           // can potentially be omitted
    },
    "argument 2": { /* ... */ },
    // ...
    "description": "..."                // can potentially be omitted
}
```

For a method with different implementations, the description looks a bit more sophisticated:
```json
"method name": {
    "methods": ["f1", "f2", /* ... */],
    "f1": { /* ... */ },                    // method description like above
    "f2": { /* ... */ },
    "recommended": /* ... */,               // e.g. "recommended" = "f1"
    "description": "..."                    // can potentially be omitted
}
```

Some methods may not contain any parameters and only a short description.

### Constraints
The constraints typically define a (potentially unclosed) domain of values like
```json
"min": 0
"max": 255
```
Whereas the `step` key defines the step size in which to move inside this domain.

E.g., kernel sizes in `{3, 6, 9, ..., 15}` can be expressed like following:
```json
"kernel": {
    "min": 3,
    "max": 15,
    "step": 3,
}
```

# Calling the pipeline
Inside `pipeline.py` we provide all methods of the pipeline.
These methods take an image and a dictionary as arguments.

In order to go from a dict of pipeline arguments to the targeted methods, the global dict argument must have the following form:
```json
{
    "img": /* ... */,
    "pipeline": {
        "method": {
            // ... arguments
        }
    }
}
```