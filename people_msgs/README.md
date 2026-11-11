## At A Glace

[people_msgs/msg/PositionMeasurement](https://docs.ros.org/en/rolling/p/people_msgs/msg/PositionMeasurement.html) -
Generic message for detecting the location of a person. Includes a 3-D
point estimate, reliability estimate and covariance matrix.

[people_msgs/msg/PositionMeasurementArray](https://docs.ros.org/en/rolling/p/people_msgs/msg/PositionMeasurementArray.html) -
An array of !PositionMeasurements as well as a co-occurence array.

[people_msgs/msg/Person](https://docs.ros.org/en/rolling/p/people_msgs/msg/Person.html) -
A position estimate with an added velocity vector, as well as the
ability to add tags to the person. Has no header, should only be used in
conjunction with !PersonStamped or People.

[people_msgs/msg/PersonStamped](https://docs.ros.org/en/rolling/p/people_msgs/msg/PersonStamped.html) -
A header + a person

[people_msgs/msg/People](https://docs.ros.org/en/rolling/p/people_msgs/msg/People.html) -
A header + an array of persons
