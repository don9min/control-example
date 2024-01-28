# control-example
Examples of MPC, PID, Max-Min Control Techniques

## Define the following dynamic system
1) Let $x^{'}_1 = x_2$. That is, $x_1$ is the derivative of time $t$.
 Here, $x_1$ is called position, and $x_2$ is velocity.
2) Let $x^{'}_2 = u$. Here, $u$ is the input to the system, meaning the input is the rate of change of velocity. $u$ can be interpreted as acceleration or 'force divided by mass'.
3) At this time, let $z(t) = x_1(t)$ be the tracking objective.
4) The input $u$ applied to the system varies over time.

Assume the following additional constraints:
1) $x_1(t)$ is less than or equal to 1.01 for all times $t$.
2) The absolute value of $x_2(t)$ is less than or equal to 0.2.
