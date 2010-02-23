The file learning_curve_points lists the points along the learning curve that the
learning curve script should visit.
Eaach point is of the form -${P} where P is a multiple of 0.1 between 0.1 and 1.0
that specifies what proportion of the training set should be used. (The entirety
of the test set is alway used)
The purpose of this file is so that you can control the script more flexibly. For
example, during debugging, you may wish to run point 0.1 first. When evaluating
an experimental algorithm that you're fairly certain won't crash, you may wish to
run 1.0 first so you can see the best results ASAP.
