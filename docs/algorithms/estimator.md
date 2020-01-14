# Estimator

## Introduction

The estimator is used to calculate an estimate of the attitude and angular velocity of the multirotor.  It is assumed that the flight controller is mounted rigidly to the body of the aircraft (perhaps with dampening material to remove vibrations from the motors), such that measurements of the on-board IMU are consistent with the motion of the aircraft.

Due to the limited computational power on the embedded processor, and to calculate attitude estimates at speeds up to 8000Hz, a simple complementary filter is used, rather than an extended Kalman filter.  In practice, this method works extremely well, and it used widely throughout commercially available autopilots.  There are a variety of complementary filters, but the general theory is the same.  A complementary filter is a method that combines low and high frequency data (complementary in frequency bandwidth).  It can be used to fuse the measurements from a gyroscope, accelerometer and sometimes magnetometer to produce an estimate of the attitude of the MAV.


## Complementary Filtering
The idea behind complementary filtering is to try to get the "best of both worlds" of gyros and accelerometers.  Gyros are very accurate in short spaces of time, but they are subject to low-frequency drift.  Accelerometers don't drift in the long scheme of things, but they experience high-frequency noise as the MAV moves about.  So, to solve these problems, the complementary filter primarily propagates states using gyroscope measurements, but then corrects drift with the accelerometer, which is a partial source of attitude measurements.  In a general sense, it is like taking a high-pass filtered version of gyroscope measurements, and a low-pass filtered version of accelerometers, and fusing the two together in a manner that results in an estimate that is stable over time, but also able to handle quick transient motions.

For an excellent review of the theory of complementary filtering, consult Mahony's Nonlinear Complementary Filtering on SO(3) paper[^1].

## Attitude Representation
There are a number of ways to represent the attitude of a MAV.  Often, attitude is represented in terms of the Euler angles yaw, pitch and roll, but it can also be represented in other ways, such as rotation matrices, and quaternions.

### Euler Angles
Euler angles represent rotations about three different axes, usually, the z, y, and x axes in that order.  This method is often the most easy for users to understand and interpret, but it is by far the least computationally efficient.  To propagate euler angles, the following kinematics are employed:

$$ \begin{equation}
	\begin{bmatrix}
		\dot{\phi} \\
		\dot{\theta} \\
		\dot{\psi} \\
	\end{bmatrix}
	=
	\begin{bmatrix}
		1 & \sin(\phi) \tan(\theta) & \cos(\phi)\tan(\theta) \\
		0 & \cos(\phi)              & -\sin(\phi) \\
		0 & \frac{\sin(\phi)}{\cos(\theta)} & \frac{\cos(\phi)}{\cos(\theta)}  \\
	\end{bmatrix}
	\begin{bmatrix}
		p \\
		q \\
		r \\
	\end{bmatrix}
\end{equation} $$


Note the large number of trigonometric functions associated with this propagation.  In a complementary filter, this will be evaluated at every measurement, and the non-linear coupling between $\omega$  and the attitude becomes very expensive, particularly on embedded processors.

Another shortcoming of euler angles is known as "gimbal lock".  Gimbal lock occurs at the "singularity" of the euler angle representation, or pitched directly up or down.  The problem occurs because there is more than one way to represent this particular rotation.  There are some steps one can take to handle these issues, but it is a fundamental problem associated with using euler angles, and motivates the other attitude representations.

### Rotation Matrix

Rotation matrices, are often used in attitude estimation, because they do not suffer from gimbal lock, are quickly converted to and from euler angles, and because of their simple kinematics.

$$
\begin{equation}
	\dot{R} = \lfloor\omega\rfloor R
\end{equation}
$$

where \(\lfloor\omega\rfloor\) is the skew-symmetric matrix of \(\omega\), and is related to calculating the cross product.

$$
\begin{equation}
	\lfloor\omega\rfloor =
	\begin{bmatrix}
		0 & -r & q \\
		r & 0 & -p \\
		-q & p & 0
	\end{bmatrix}
\end{equation}
$$

This propagation step is linear with respect to the angular rates, which simplifies calculation significantly.

A rotation matrix from the inertial frame to body frame can be constructed from euler angles via the following formula:

$$
\newcommand{\ct}{c\theta}
\newcommand{\cp}{c\phi}
\newcommand{\cs}{c\psi}
\newcommand{\st}{s\theta}
\newcommand{\sphi}{s\phi}
\newcommand{\spsi}{s\psi}
\begin{equation}
	    R = \begin{bmatrix}
	    	\ct\cs & \ct\spsi & -\st \\
        \sphi\st\cs-\cp\spsi & \sphi\st\spsi+\cp\cs & \sphi\ct \\
        \cp\st\cs+\sphi\spsi & \cp\st\spsi-\sphi\cs & \sphi\st \\
        \end{bmatrix}
\end{equation}
$$

and converting back to euler angles is done via the following;

$$
\begin{equation}
  \begin{bmatrix}
    \phi \\
    \theta \\
    \psi \\
  \end{bmatrix} =
	\begin{bmatrix}
	 \textrm{atan2}\left(R_{32}, R_{33}\right) \\
   \textrm{atan2}\left(-R_{31}, \sqrt{R_{21}^2 + R_{33}^2}\right) \\
   \textrm{atan2}\left(R_{21}, R_{11}\right) \\
  \end{bmatrix}
\end{equation}
$$


### Quaternions

Quaternions are a number system which extends complex numbers.  They have four elements, commonly known as \(w\), \(x\), \(y\), and \(z\).  The last three elements can be though of as describing an axis, \(\beta\) about which a rotation occurred, while the first element, \(w\) can be though of as describing the amount of rotation \(\alpha\) about that axis. (see eq~\ref{eq:euler_to_axis_angle}). While this may seem straight-forward, quaternions are normalized so that they can form a group.  (That is, a quaternion multiplied by a quaternion is a quaternion), so they end up being really difficult for a human being to interpret just by looking at the values.  However, they provide some amazing computational efficiencies, most of which comes from the following special mathematics associated with quaternions.

First, just the definition of a quaternion:

$$
\begin{equation}
	q = \begin{bmatrix}
				q_w \\
				q_x \\
				q_y \\
				q_z
			 \end{bmatrix}
		   = \begin{bmatrix}
				\cos(\alpha/2) \\
				\sin(\alpha/2)\cos(\beta_x) \\
				\sin(\alpha/2)\cos(\beta_y) \\
				\sin(\alpha/2)\cos(\beta_y)
			 \end{bmatrix}
			 = \begin{bmatrix}
			   s \\
			   v_x \\
			   v_y \\
			   v_z
			 \end{bmatrix}
	\label{eq:euler_to_axis_angle}
\end{equation}
$$

and second, some formulas to convert to and from euler angles.

$$
\begin{equation}
	q =
	  \begin{bmatrix}
	  	\cos(\theta/2)\cos(\theta/2)\cos(\psi/2) + \sin(\phi/2)\sin(\theta/2)\sin(\psi/2) \\
	  	\sin(\theta/2)\cos(\theta/2)\cos(\psi/2) - \cos(\phi/2)\sin(\theta/2)\sin(\psi/2) \\
	  	\cos(\theta/2)\sin(\theta/2)\cos(\psi/2) + \sin(\phi/2)\cos(\theta/2)\sin(\psi/2) \\
	  	\cos(\theta/2)\cos(\theta/2)\sin(\psi/2) - \sin(\phi/2)\sin(\theta/2)\cos(\psi/2) \\
	  \end{bmatrix}
\end{equation}
$$

$$
\begin{equation}
	\begin{bmatrix}
		\phi \\
		\theta \\
		\psi \\
	\end{bmatrix}
	=
	\begin{bmatrix}
		\textrm{atan2}\left( 2\left(q_wq_x + q_yq_z\right),1-2\left(q_x^2+q_y^2\right) \right) \\
		\textrm{sin}^{-1}\left( 2 \left( q_wq_y - q_zq_x \right)  \right) \\
		\textrm{atan2}\left( 2 \left( q_wq_z + q_xq_y \right), 1 - 2 \left( q_y^2 q_z^2 \right)  \right)
	\end{bmatrix}
	\label{eq:euler_from_quat}
	\tag{5}
\end{equation}
$$

The quaternion group is "closed" under the following operation, termed quaternion multiplication.

$$
q_1 \otimes q_2 = \begin{bmatrix} s_1s_2 - v_1^\top v_2 \\ s_1v_2 + s_2v_1 + v_1 \times v_2 \end{bmatrix}
$$

To take the "inverse" of a quaternion is simply to multiply $s$ or $v$ by $-1$

\begin{equation}
	q^{-1} = \begin{bmatrix}
				-q_w \\
				q_x \\
				q_y \\
				q_z
			 \end{bmatrix}
			 	= \begin{bmatrix}
				q_w \\
				-q_x \\
				-q_y \\
				-q_z
			 \end{bmatrix}
\end{equation}

and to find the difference between two quaternions, just quaternion multiply the inverse of one quaternion with the other.

\begin{equation}
	\tilde{q} = \hat{q}^{-1} \otimes q
\end{equation}

However, the most important aspect of quaternions is the way we propagate dynamics.

\begin{equation}
	\dot{q} = \frac{1}{2} q \otimes q_\omega
\end{equation}

where \(q_\omega\) is the pure quaternion created from angular rates.

\begin{equation}
	q_\omega = \textrm{p}(\omega) =
			  \begin{bmatrix}
				0 \\
				p \\
				q \\
				r
			 \end{bmatrix}
\end{equation}

What this means is that, like rotation matrices, quaternion dynamics are _linear_ with respect to angular rates, as opposed to euler angles, which are non-linear, and they take less computation than rotation matrices because they have fewer terms.  Casey et al. [Casey2013] performed a study comparing all three of the above representations, and found that complementary filters using an euler angle representation took **12 times** longer to compute on average than a quaternion-based filter.  Quaternions were also about 20% more efficient when compared with rotation matrices.  For these reasons, ROSflight uses quaternions in its filter.

## Derivation

ROSflight implements the quaternion-based passive "Mahony" filter as described in [this paper](https://hal.archives-ouvertes.fr/hal-00488376/document) [^1].  In particular, we implement equation 47 from that paper, which also estimates gyroscope biases.  A Lyuapanov stability analysis is performed in that paper, in which it is shown that all states and biases, except heading, are globally asymptotically stable given an accelerometer measurement and gyroscope.  The above reference also describes how a magnetometer can be integrated in a similar method to the accelerometer.  That portion of the filter is omitted here due to the unreliable nature of magnetometers on-board modern small UAS.

### Passive Complementary Filter
The original filter propagates per the following dynamics:

$$
\newcommand{\werr}{\omega_\text{err}}
\newcommand{\wfinal}{\omega_\text{final}}
\begin{equation}
	\begin{aligned}
	\dot{\hat{q}} &= \frac{1}{2} \hat{q} \otimes \textrm{p}\left(\wfinal\right) \\
	\dot{\hat{b}} &= -2k_I\werr
	\end{aligned}
	\label{eq:traditional_prop}
	\tag{1}
\end{equation}
$$

where \(\textrm{p}\left(\cdot\right)\) creates a pure quaternion from a 3-vector. The term \(\wfinal\) is a composite angular rate which consists of the measured angular rates, \(\bar{\omega}\), the estimated gyroscope biases, \(\hat{b}\), and a correction term calculated from another measurement of attitude (usually the accelerometer), \(\werr\). The constant gains \(k_p\) and \(k_I\)s are used in determining the dynamics of the filter.
$$
\begin{equation}\label{eq:q_omega}\tag{2}
	\wfinal = \bar{\omega} - \hat{b} + k_P\werr
\end{equation}
$$

$$
\newcommand{\avec}{a}
\newcommand{\gvec}{g}
\newcommand{\qmeas}{q_{acc}}
\newcommand{\gamvec}{\gamma}
\newcommand{\norm}[1]{\left\lVert#1\right\rVert}
\newcommand{\wbar}{\bar\omega}
\newcommand{\w}{\omega}
\newcommand{\qhat}{\hat{q}}
$$

The correction term \(\werr\) can be understood as the error in the attitude as predicted by another source (e.g., the accelerometer).  To calculate \(\werr\) the quaternion describing the rotation between the accelerometer estimate and the z-axis of the inertial frame (i.e., where gravity *should* be), \(\qmeas\), is first calculated

$$
\begin{equation}
	\avec =
		\begin{bmatrix}
			a_x \\
			a_y \\
			a_z \\
			\end{bmatrix},
	\qquad
	\gvec =
	  \begin{bmatrix}
			0 \\
			0 \\
			1 \\
		\end{bmatrix},
	\qquad
	\gamvec = \frac{\avec+\gvec}{\norm{\avec+\gvec}},
	\qquad
	\qmeas =
	  \begin{bmatrix}
	    \avec^\top \gamvec \\
	    \avec \times \gamvec
	  \end{bmatrix}
\end{equation}
$$

$$
\newcommand{\qtilde}{\tilde{q}}
\newcommand{\vtilde}{\tilde{v}}
$$

Next, the quaternion error between the estimate \(\qhat\) and the accelerometer measure \(\qmeas\) is calculated.
\begin{equation}
	\qtilde = \qmeas^{-1} \otimes \qhat =
		\begin{bmatrix}
			\tilde{s}\\
			\vtilde
		\end{bmatrix}
\end{equation}

Finally, \(\qmeas\) is converted back into a 3-vector per the method described in eq. 47a of Mahony2007 [^1].
\begin{equation}
	\werr = 2\tilde{s}\vtilde
\end{equation}

Both the attitude quaternion and bias dynamics can be integrated using standard Euler integration, requiring that the resulting quaternion is re-normalized.

### Modifications to Original Passive Filter
There have been a few modifications to the passive filter described in Mahony2007 [^1], consisting primarily of contributions from [Casey2013](https://arc.aiaa.org/doi/pdf/10.2514/6.2013-4615).  Firstly, rather than simply taking gyroscope measurements directly as an estimate of $\omega$, a quadratic polynomial is used to approximate the true angular rate from gyroscope measurements to reduce error.  In Casey2013, this process was shown to reduce RMS error by more than 1,000 times.  There are additional steps associated with performing this calculation, but the benefit in accuracy more than compensates for the extra calculation time.  The equation to perform this calculation is shown in Eq.\(\eqref{eq:quad_approx}\).

\begin{equation}
	\bar{\omega} = \frac{1}{12}\left(-\omega\left(t_{n-2}\right) + 8\omega\left(t_{n-1}\right) + 5\omega\left(t_n\right) \right)
	\label{eq:quad_approx}
	\tag{4}
\end{equation}

where \(\omega(t_{n-x})\) is the gyroscope measurement of the angular rate \(x\) measurements previous.

The second modification is in the way that the attitude is propagated after finding \(\dot{\hat{q}}\).  Instead of performing the standard euler integration

$$
\begin{equation}
	\hat{q}_n = \hat{q}_{n-1}+\dot{\hat{q}}_n dt
\end{equation}
$$


we use an approximation of the matrix exponential.  The matrix exponential arises out of the solution to the differential equation \(\dot{x} = Ax\), namely
$$
\begin{equation}
	x(t) = e^{At} x(0)
\end{equation}
$$

and the discrete-time equivalent
$$
\begin{equation}
	x(t_{n+1}) = e^{hA}(t_n)
\end{equation}
$$

This discrete-time matrix exponential can be approximated by first expanding the matrix exponential into the infinite series

\begin{equation}
	e^{A} = \sum_{k=0}^\infty \dfrac{1}{k!}A^k
\end{equation}


and then grouping odd and even terms from the infinite series into two sinusoids, and results in the following equation for the propagation of the quaternion dynamics

$$
\begin{equation}
	\qhat(t_n) = \left[ \cos \left( \frac{\norm{\w}h}{2}\right)I_4  + \frac{1}{\norm{\w}} \sin \left( \frac{\norm{\w}h}{2} \right) \lfloor\w\rfloor_4   \right] \qhat(t_{n-1})
	\label{eq:mat_exponential}
	\tag{3}
\end{equation}
$$

where \(\lfloor\w\rfloor_4\) is the 4x4 skew-symmetric matrix formed from \(\w\)

\begin{equation}
	\lfloor\w\rfloor_4 =
	\begin{bmatrix}
	    0	& - p & - q  & - r  \\
      	p   &   0   &  r & - q \\
      	q   & - r  &  0   &  p \\
      	r   &  q  & - p & 0  \\
	\end{bmatrix}
\end{equation}

## External Attitude Measurements
Using the ROSflight estimator with gyro measurements only will quickly drift due to gyro biases. The accelerometer makes the biases in \(p\) and \(q\) observable and provides another measurement of pitch and roll. To make yaw observable, an external attitude measurement can be provided to the estimator which is used in much the same way as the accelerometer. Instead of as outlined above for accelerometer updates, the correction term \(\werr\) can be calculated as
$$
\begin{equation}
	\werr = k_\text{ext}\sum_{i=1}^3 R(\hat{q})^\top e_i\times \bar{R}^\top e_i
\end{equation}
$$
where \(k_\text{ext}= F_s^\text{IMU} / F_s^\text{ext}\) is the ratio of the IMU sample rate to the external attitude sample rate.

In our implementation, whenever an external attitude measurement is supplied, if there was a \(\werr\) calculated from the accelerometer, it is overwritten by the above calculation for an external attitude update. Also note that the gain \(k_P\) associated with an external attitude can be much higher if we trust the source of the external attitude measurement.


## Tuning
The filter can be tuned with the two gains \(k_P\) and \(k_I\).  Upon initialization, \(k_P\) and \(k_i\) are set very high, so as to quickly cause the filter to converge upon approprate values.  After a few seconds, they are both reduced by a factor of 10, to a value chosen through manual tuning.  A high $k_P$ will cause sensitivity to transient accelerometer errors, while a small $k_P$ will cause sensitivity to gyroscope drift.  A high $k_I$ will cause biases to wander unnecessarily, while a low $k_I$ will result in slow convergence upon accurate gyroscope bias estimates.  These parameters generally do not need to be modified from the default values.

## Implementation
The entire filter is implemented in float-based quaternion calculations.  Even though the STM32F10x microprocessor does not contain a floating-point unit, the entire filter has been timed to take about 370\(\mu\)s.  The extra steps of quadratic integration and matrix exponential propagation can be ommited for a 20\(\mu\)s and 90\(\mu\)s reduction in speed, respectively.  Even with these functions, however, this is sufficiently short to run at well over 1000Hz, which is the update rate of the MPU6050 on the naze32.

Control is performed according to euler angle estimates, and to reduce the computational load of converting from quaternion to euler angles (See Equation\(\eqref{eq:euler_from_quat}\)), a lookup table approximation of atan2 and asin are used.  The Invensense MPU6050 has a 16-bit ADC and an accelerometer and gyro onboard.  The accelerometer, when scaled to \(\pm\)4g, has a resolution of 0.002394 m/s\(^2\).  The lookup table method used to approximate atan2 and asin in the actual implementation is accurate to \(\pm\) 0.001 rad.  Given the accuracy of the accelerometer, use of this lookup table implementation is justfied.  The C-code implementation of the estimator can be found in the file `src/estimator.c.`

[^1]: Mahony, R., Hamel, T. and Pflimlin, J. (2008). Nonlinear Complementary Filters on the Special Orthogonal Group. IEEE Transactions on Automatic Control, 53(5), pp.1203-1218.
