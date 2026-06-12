# Simulation of Urban Automotive Radar Measurements for Deep Learning Target Detection

Thomas Wengerter1, Rodrigo Perez ´ 2, Erwin Biebl2, Josef Worms1 and Daniel O´Hagan1

Abstract— Frequency modulated continuous wave radars are an important component of modern driver assistance systems and enable safer automated driving. To achieve real time detection and classification of multiple road users in the range-Doppler map, the usage of neural target detection networks is proposed. Since the amount of labelled radar measurements available limits the training process, a new radar simulation framework is presented which generates arbitrary traffic scenarios with reflection models for pedestrians, bicyclists and vehicles. With an adaptive FMCW setup, sequences of dynamic urban multi-target radar measurements are simulated, maintaining minimum computational complexity. Solely trained on simulated measurement data, the neural network achieves an average precision above 87% on bicyclists and vehicles in real measurement data which is comparable to the performance of neural networks trained on real measurement datasets.

## I. INTRODUCTION

Self-driving cars require accurate and fast road user detection to provide real time scene understanding and navigation. Radars are a cheap sensor solution and excel by measuring distance, relative angle and radial velocity of targets simultaneously. However, analyzing radar measurements of complex urban scenarios with multiple, even partially obstructed road users remains challenging. This paper addresses the process of automotive radar target detection and aims to improve the detection performance.

From electrically large targets, reflections are scattered back to the radar receiver from many different positions on the surface and appear as a cloud or cluster in the 3 dimensional (3D) radar cube with range, azimuth and Doppler velocity. Commonly, two-stage approaches use clustering algorithms to extract regions of interest from the range-Doppler spectrum and then associate each region of interest to a single target class [1]. A one-stage neural radar target detection network based on EfficientDet [2] is proposed to locate and classify road users directly on the low level reflection spectra, avoiding error-prone clustering steps.

Training success of deep neural networks depends heavily on the size and diversity of the labelled dataset recorded with the given radar setup. Collecting enough data from test drives and labelling is a time consuming task. Therefore, a full radar measurement simulation is introduced which generates intermediate frequency (IF) signals synthetically for multitarget scenarios with moving cars, pedestrians and bicycles.

By using multi-point reflector target models, dynamic multitarget scenarios with carrier frequencies around 76 GHz become computationally accessible, which are currently not practical to solve in electromagnetic ray tracing solvers. Thus, testing data for different radar system setups or special use cases can be generated easily with accurate labels.

This paper presents the developed simulation models and the neural target detection network. Finally, the detection performance of the neural network trained with solely simulated data is tested on real radar measurement data to validate the quality of the proposed simulation models. Furthermore, the detector’s robustness against partial target occlusion is investigated.

## II. RELATED WORK

Traditional target detection procedures use peak detection with constant false alarm rate (CFAR) algorithms like OS-CFAR [3], followed by point target clustering algorithms like DBSCAN [1] for detection and pattern recognition for classification [4], [5]. However, CFAR and clustering algorithms struggle with dynamic clutter and complex multitarget scenarios where targets might overlap [6].

Deep neural networks perform well as target detector on range profiles or range-Doppler maps, outperforming the conventional CFAR algorithms in dynamic clutter environments [7]. Detecting and classifying target signatures directly from the radar cube using large convolutional neural networks (CNN) achieves promising results on real radar measurement data. In [8], vehicles are detected directly from the 3D radar cube input and in [6], target level information in combination with a cropped window of the radar cube as input results in accurate classification for vehicles, bicyclists and pedestrians. These publications use measurement data recorded on test drives to generate their training and testing data which are labelled by object detection on the camera image. On the other hand, precise simulations for automotive traffic environments based on ray-tracing, geometrical and physical optics are known from full 3D electromagnetic (EM) simulation environments like Ansys or CST Studio. Generating large datasets with multi-target urban traffic scenarios is hardly feasible due to their simulation complexity. The radar characteristics of vulnerable road users are already investigated thoroughly and the measurement observations propose that range-Doppler signatures of pedestrians [9], bicycles [10] and cars [11] can be associated explicitly with their moving body parts. Dynamic reflection point target models are presented in [10] and [9] to emulate the most prominent radar signatures of bicyclists and pedestrians.

A similar physics-based simulation approach of automotive multi-target scenarios is presented in [12] lacking the micro-Doppler signatures of dynamic targets.

Therefore, a new point target model for cars is proposed. The bicyclist and pedestrian simulation model are improved by sampling the reflection points from the target surfaces for higher model dynamics, eliminating hidden reflection points and projecting reflection points on a horizontal plane parallel to the ground. All target models are integrated to an adaptive simulation framework generating continuous radar measurements of dynamic urban traffic scenarios. This report about training a neural detection network on purely simulated data and testing on real measurement data delivers new insights to the relevance of simulated radar data for developing and testing radar detection systems in future.

## III. RADAR SIMULATION FOR MULTI-TARGET AUTOMOTIVE SCENARIOS

## A. Baseband Signal Simulation

To achieve reasonable performance, the frequency modulated continuous wave (FMCW) radar simulation generates the baseband signals corresponding to the given measurement scenario. This approach avoids having to work with high sampling rates in the W-band.

Assume a stationary radar with a linear frequency modulated chirp

$$
s _ { \mathrm { T X } } ( t ) = A _ { \mathrm { T X } } \exp \left[ j 2 \pi t \left( f _ { 0 } + \frac { B } { T _ { \mathrm { P } } } t \right) \right]\tag{1}
$$

in the periodic FMCW transmitter signal with amplitude $A _ { \mathrm { T X } } ,$ carrier frequency $f _ { 0 } ,$ chirp bandwidth B and time $t \in \ [ 0 , T _ { \mathrm { P } } ]$ Transmitted through an ideal channel, signal $s _ { \mathrm { T X } } ( t )$ travels to a point reflector at distance

$$
R = \| \mathbf { r } \| = { \sqrt { x ^ { 2 } + y ^ { 2 } + z ^ { 2 } } } .\tag{2}
$$

After scattering reflection, a replicate of transmitted signal arrives at the receiver, delayed by

$$
\tau = { \frac { 2 R } { c _ { 0 } } }\tag{3}
$$

with a constant phase shift

$$
\Phi = { \frac { 4 \pi R } { \lambda } } = { \frac { 2 R f _ { 0 } } { c _ { 0 } } } ,\tag{4}
$$

where λ denotes the wavelength and $c _ { 0 }$ the speed of light. If the point reflector is moving with a constant velocity vector $\mathbf { v } ,$ the reflected signal $s ( t )$ is additionally modulated by a Doppler frequency. The Doppler frequency derives from the radial velocity component $v _ { \mathrm { r } }$ and is

$$
f _ { \mathrm { D } } = - 2 v _ { \mathrm { r } } { \frac { f _ { 0 } } { c _ { 0 } } } .\tag{5}
$$

At the FMCW radar receiver, the baseband signal results from mixing the transmitted chirp signal with the received signal and removing the high frequency filter products by

low pass filtering. The resulting intermediate frequency (IF) signal is the baseband signal

$$
s _ { \mathrm { b } } ( t ) = A _ { \mathrm { R X } } \exp \left[ - j 2 \pi \left( - \frac { 2 v _ { \mathrm { r } } f _ { 0 } } { c _ { 0 } } - \frac { 2 B R } { T _ { \mathrm { c } } c _ { 0 } } \right) t - \frac { 2 R f _ { 0 } } { c _ { 0 } } \right]\tag{6}
$$

comprising the range delay in (3), the Doppler shift in (5) and the constant phase shift in (4). The received signal’s amplitude $A _ { \mathrm { R X } }$ can be estimated from the signal power at the receiver, antenna gains and the point scatterer’s radar cross section (RCS) with the fundamental radar power equation.

For a quick simulation of road users, models with groups of point reflectors as in (6) are used. All reflections interfere at the radar receiver, so the full baseband signal is determined by superposition of all baseband signals

$$
s _ { \flat } ( t ) = \sum _ { i = 1 } ^ { \mathrm { T } } s _ { \flat , i } ( t ) .\tag{7}
$$

A conventional simulation of an uniform linear antenna array at the receiver is included in the simulations to determine the antenna gain and measure the angle of arrival from the signals’ phase shifts. Antenna arrays at the transmitter are simplified to a single antenna element with equivalent gain.

Besides the targets, the radar receives reflections from its static environment (clutter), which are simulated by random static point reflections. Additive white gaussian noise is added to the signals, weighted by a power scaling function which is adding a exponential noise floor decrease with the delay. Both clutter and noise levels are controllable. If the radar is also moving, its relative movement is considered in the equations for range (2) and Doppler (5). Finally, 3 consecutive fast fourier transforms (FFT) are applied to span the 3D radar cube: First over the time signals, second over the individual chirps and third over the RD spectra of the antenna elements. The radar cube is converted to decibel and serves as input to the neural detection network.

## B. Simulation Models for Vulnerable Road Users

With the high frequency radar sensors, wavelengths of a few millimetre lengths are illuminating the road users, so the targets are considered electrically large objects. Due to wave propagation on an electrically large target’s surface, its reflections are mainly scattered from few dominant scattering points. Measurement observations of road users indicate that radar signatures of pedestrians [9], bicycles [10] and cars [11] can be associated explicitly with their moving body parts. Conform to collected measurement results, prominent reflecting areas on the individual targets are identified and represented by individually moving point scatterers in the model, associated with the measured RCS. To boost the final simulation performance, the whole radar measurement scenario is reduced to a 2 dimensional scene by projecting all point reflectors from the 3 dimensional target models on a horizontal plane following the radar’s line of sight parallel to the ground. Each reflection point features a projected radial velocity for realistic micro-Doppler signatures.

1) Point Model for Pedestrians: The point target model for pedestrians is based on the micro-Doppler signature analysis of moving body parts given in [9], [13]. The pedestrian is simulated by 12 individually moving scattering points for feet, knees, elbows, shoulders, hands, head and torso, projected on the horizontal plane parallel to the ground. Both RCS and reflection point walking movements of each body part are presented in [13]. The given time velocity functions are scaled to the simulated walking speed, assigned to the point scatterers and projected on the horizontal plane.

2) Point Model for Vehicles: Reflection characteristics of different vehicles are analyzed in [11], [14]. Both illustrate that the most amount of reflections recorded from a vehicle occur in a distance of 15 cm from the car’s facing outer contour. Also, the wheel arches cause strong reflections. Inspirations for the chosen solution are found in the vehicle models in [15], [16] and [17]. A variable multi-gaussian model is modelled around the car’s contours to sample the reflection positions from a probability distribution approximating the measured reflection heatmaps in [16]. As illustrated in Fig. 1, two probability distributions along the local coordinate system’s xi and yi axis are imposed at the vehicle’s contour facing the radar. The distance between two contour points equals the radar’s range resolution $\frac { c _ { 0 } } { 2 \ : B }$ , and smaller at the corners. To determine the coordinates of the actual reflection points in the horizontal plane, an arbitrary number of reflection points are sampled from the probability distribution in Fig. 1. These probability distributions stem from each illuminated contour point, which are colored red in the image. On axis yi around the contour point, a Gaussian probability density function with mean 0 and variance equal to half of the radar’s range resolution $\frac { c _ { 0 } } { 4 B }$ is sampled. Facing into the vehicle body along $x i ,$ , the Rayleigh probability density function

$$
f ( x i | \sigma _ { \mathrm { R } } ) = { \left\{ \begin{array} { l l } { { \displaystyle { \frac { x i } { \sigma _ { \mathrm { R } } ^ { 2 } } } e ^ { - { \frac { x i ^ { 2 } } { 2 \sigma _ { \mathrm { R } } ^ { 2 } } } } } } & { x i \geq 0 } \\ { 0 } & { x i < 0 } \end{array} \right. }\tag{8}
$$

with $\sigma _ { \mathrm { R } } ~ = ~ 0 . 1 5$ m has its maximum around 15 cm inside the vehicle body and decreases with larger distance to the contour point.

Special consideration is given to the wheel reflections, which dominate the heatmaps in the measurements in [14]. Reflections from the wheel cases are sampled separately from the car body with a larger number of reflections, even from the non-facing tires of the vehicle. One can easily derive that the maximum range of radial velocity must be between [0, 2vb cos θ], where $v _ { \mathrm { b } }$ is the car’s bulk velocity and θ the radar’s horizontal viewing angle relative to the wheel axis. To simulate micro-Doppler, each sample gets assigned a horizontal velocity component derived from its yi-position on the wheel as shown in Fig. 2.

Finally, the sampled point reflectors’ RCS are scaled depending on their position on the car body or wheel and the viewing angle to the radar, with total RCS as measured in [18].

<!-- image-->  
Fig. 1: Probability density functions (PDFs) for reflection point locations sampled from one contour point illuminated by the radar beam in an angle θ. The contour points are located on the outline of the car and plotted from top down view. Red points are the vehicle’s illuminated contour points, blue points the occluded contour points, green points are the sampled reflections from the probability distribution and yellow points are the reflections sampled from the wheel cases’ distributions with micro-Doppler component. The plots are not to scale.

<!-- image-->  
(a)

<!-- image-->  
(b)  
Fig. 2: Derivation of the horizontal speed $v _ { \mathrm { h o r } }$ of points on a turning car wheel with radius r. The car is moving at a speed of $v _ { b }$ . The local coordinate system placed at the wheel’s axis.

3) Point Model for Bicyclists: Compared to the pedestrian, the model for bicyclists includes additional strong reflections from the frame and the rotating wheels. In [19], the bicyclist’s range-Doppler patterns associated with the moving body or bicycle parts and the model proposed in [20] represents the frame, wheels and the bicyclist by moving point reflectors with measured RCS. Instead of this 3D model, the proposed point model simulates the bicyclist’s reflections with fewer, representative scattering points projected on the horizontal plane as shown in Fig. 3. The location of the reflection points are sampled for each measurement pulse interval from the bicyclist’s visible outlines, while hidden reflectors are omitted. Thus, the calculation overhead caused by weak or hidden reflection points is reduced significantly.

<!-- image-->  
Fig. 3: Flattened reflection model for a bicyclist, heading in in −y direction. All reflection points from the frame, the turning wheels and the bicyclist are sampled from the facing side of the bicyclist and then projected in the x-y plane.

<!-- image-->  
Fig. 4: Simulated urban radar measurement scenario with the center positions of a pedestrian (black), a bicyclist (blue) and a car (green) moving in front of the stationary radar at position (0,0). The positions are printed at every measurement interval, the total duration of this trail is 5 s.

Casting movements of the legs and rotating wheels are considered with small micro-Doppler shifts assigned to the reflections sampled from the respective area.

Modelling the road users with a group of point targets gives the opportunity to simulate partially hidden objects and test the radar on critical scenarios [11]. Each simulated target follows defined trajectories in front of the radar as shown in Fig. 4. To add complexity to the scenario, realistic acceleration, turns and deceleration are added. Due to these dynamics, the positions of the target models’ point scatterers need to be recalculated around the current center position for every measurement interval. Before the reflection signal is simulated in the new constellation, all scattering points whose line-of-sight is blocked by the 3D silhouette of another target’s scatterers are removed.

## C. Results of the Radar Target Detection Network

A set of 600 different traffic scenarios of 1 s duration are simulated for a stationary 76.5 GHz FMCW radar with bandwidth $B = 1 \mathrm { G H z } ,$ chirp duration $T _ { \mathrm { c h i r p } } = 3 2 \mu \mathrm { s }$ , pulse repetition interval $T _ { \mathrm { i n t e r v a l } } = 6 4 \mu \mathrm { s }$ , sampling frequency $f _ { s } =$ 10 MHz and a coherent processing interval of 256 chirps. One of these simulated RD map with one pedestrian, one bicyclist and one car is shown in Fig. 5. Since the focus is on multi-target scenarios, the simulation comprises up to 6 randomly positioned targets in a scenario, maximum 2 instances of each target class. The stationary FMCW simulation matches the available measurement dataset of the Radarbook radar [21] with identical setup parameters. It has limited accuracy due to the half-automatic labelling from camera images and is used as a validation set with a total duration of 70 s.

<!-- image-->  
Fig. 5: Simulated RD map of a simulated radar measurement from a stationary radar at position $( x , y , z ) = ( 0 , 0 , 0 )$ , with a pedestrian at position (5, 0, 0) moving with a radial velocity of 5 km/h, a bicyclist at position $( 1 0 , - 2 , 0 )$ moving with 24 km/h and a car at position (20, 1, 0) moving with 43 km/h. The radar is facing along the x-axis.

For demonstration, the absolute value of the simulated radar cubes are summed over the azimuth dimension, which is equivalent to summing over all antenna elements. The resulting RD spectra are converted to a grayscale image and serve as training data for the image object detection network EfficientDet. EfficientDet’s D0 compound scaling achieves the same detection accuracy as a YOLOv3 algorithm [22], but requires only 3.5% of floating point operations per second [2]. At the output, the bounding box format contains the coordinates of the bottom left corner, box width and height in pixels, so $[ x _ { \mathrm { m i n } } , y _ { \mathrm { m i n } } , w , h ]$ , which corresponds to $\begin{array} { r } { \left[ \frac { v _ { \mathrm { r , m i n } } } { d v _ { \mathrm { r } } } , \frac { R _ { \mathrm { m i n } } } { d R } , \frac { \Delta v _ { \mathrm { r } } } { d v _ { \mathrm { r } } } , \frac { \Delta R } { d R } \right] } \end{array}$ with the Doppler velocity spread $\Delta v _ { \mathrm { r } }$ and range extension $\Delta R$ in the RD map. The range extension corresponds to the radial length of an illuminated target. Since the Doppler velocity spread is not relevant for the scene understanding, the detector is trained on fixed bounding box widths for each target class. Each bounding box comes with a set of class scores pedestrian, bicycle or vehicle in one hot coding format.

To evaluate the detection performance of the trained neural detection network, a threshold is set on the intersection over union (IoU) of the bounding boxes with the labels. Efficient-Det, only trained on simulated radar scenarios, achieves an overall average precision (AP) of 86.8% on simulated testing data and 52.8% AP on real measuement data. Cars and bicyclists are detected correctly with an overall AP of 88.0% in the real measurement dataset. The largest performance loss of the detection network appears in the pedestrian class with an AP of only 32.5%.

TABLE I: Detection AP of different target detection networks.  
\*: Trained and tested on simulated data.  
\*\*: Trained on simulated data and tested on measurement data.  
\*\*\*: Trained and tested on measurement data.
<table><tr><td>Detector</td><td>IoU All</td><td>Pedestrians</td><td>Bicyclists</td><td>Vehicles</td></tr><tr><td>EfficientDet*</td><td>0.5 86.8%</td><td>78.5%</td><td>97.2%</td><td>97.9%</td></tr><tr><td>EfficientDet**</td><td>0.2 52.8%</td><td>32.5%</td><td>87.3%</td><td>88.8%</td></tr><tr><td>YOLO*** [23]</td><td>0.570.6%</td><td>-</td><td>-</td><td>-</td></tr><tr><td>SSD*** [8]</td><td>- -</td><td>-</td><td>1</td><td>87.6%</td></tr></table>

Detection performance on pedestrians, bicyclists and vehicles are collected in Table I. As reference, the results of a YOLO for vulneralbe road user detection from [23] and the results of a SSD for vehicle detection on highways with a SSD from [8] are given. Both of the referenced detection networks are trained and evaluated on real measurement data only. Since EfficientDet trains on simulation data with even more accurate labelling than the available measurement data in the validation set, a missmatch of the bounding box positions and shapes persists in the validation. To compensate small offsets between measurement labels and simulation labels, the IoU threshold for EfficientDet is reduced to 0.2. This adjustment can become obsolete when using the real measurement data for transfer learning.

A representative detection result is shown in Fig. 6. For vehicles and bicycles, the detection works well and achieves high prediction scores. False positives for pedestrians occur around the static clutter, which impairs the AP result. Additionally, accurate labelling for pedestrians is difficult in the recorded measurement data, so the detected positions are often just slightly off the label and below the IOU threshold. This explains the low AP for pedestrians, while the validation data inspection shows that pedestrians are mostly detected correctly.

## IV. CONCLUSION

This paper presents an efficient FMCW radar measurement simulation for multi-target traffic scenarios and a real-time road user detection based on EfficientDet. The high AP above 87% for bicyclists and cars in real measurement data validates that the simulation generates realistic radar data with similar training success compared to real measurement datasets. Thus, it is suitable for transfer learning or creating test data for critical traffic scenarios at low costs. The (b): Bounding boxes after class threshold 0.5. Note that the bounding boxes from the labels have different shape and position than the detected box from the network.

<!-- image-->  
(a)

<!-- image-->  
(b)  
Fig. 6: Detection results on real measurement data, trained only with simulated data. Manually labelled ground truth bounding boxes are drawn in blue, all other boxes are detections from the network with the label and class score. (a): All detections with class score.

simulation and detection of pedestrians needs to be improved in future, also environment influences like street or guard rail reflections and multipath propagation will supplement the simulation.

## REFERENCES

[1] A. Ram, J. Sunita, A. Jalal, and K. Manoj, “A density based algorithm for discovering density varied clusters in large spatial databases,” International Journal of Computer Applications, vol. 3, 06 2010.

[2] M. Tan, R. Pang, and Q. V. Le, “Efficientdet: Scalable and efficient object detection,” in The IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR), June 2020.

[3] H. Rohling, “Radar cfar thresholding in clutter and multiple target situations,” IEEE Transactions on Aerospace and Electronic Systems, vol. AES-19, no. 4, pp. 608– 621, 1983.

[4] S. Haag, B. Duraisamy, F. Govaers, W. Koch, M. Fritzsche, and J. Dickmann, “Extended object tracking assisted adaptive clustering for radar in autonomous driving applications,” in 2019 Sensor Data Fusion: Trends, Solutions, Applications (SDF), 2019, pp. 1–7.

[5] Z. Zhao, Y. Song, F. Cui, J. Zhu, C. Song, Z. Xu, and K. Ding, “Point cloud features-based kernel svm for human-vehicle classification in millimeter wave radar,” IEEE Access, vol. 8, pp. 26 012–26 021, 2020.

[6] A. Palffy, J. Dong, J. F. P. Kooij, and D. M. Gavrila, “Cnn based road user detection using the 3d radar cube,” IEEE Robotics and Automation Letters, vol. 5, no. 2, pp. 1263–1270, 2020.

[7] R. Perez, F. Schubert, R. Rasshofer, and E. Biebl, ´ “Range detection on time-domain fmcw radar signals with a deep neural network,” IEEE Sensors Letters, vol. 5, no. 2, pp. 1–4, 2021.

[8] B. Major, D. Fontijne, A. Ansari, R. T. Sukhavasi, R. Gowaikar, M. Hamilton, S. Lee, S. Grzechnik, and S. Subramanian, “Vehicle detection with automotive radar using deep learning on range-azimuth-doppler

tensors,” in 2019 IEEE/CVF International Conference on Computer Vision Workshop (ICCVW), 2019, pp. 924–932.

[9] L. Vignaud, A. Ghaleb, J. L. Kernec, and J. Nicolas, “Radar high resolution range micro-doppler analysis of human motions,” in 2009 International Radar Conference ”Surveillance for a Safer World” (RADAR 2009), 2009, pp. 1–6.

[10] E. Schubert, F. Meinl, M. Kunert, and W. Menzel, “High resolution automotive radar measurements of vulnerable road users – pedestrians cyclists,” in 2015 IEEE MTT-S International Conference on Microwaves for Intelligent Mobility (ICMIM), 2015, pp. 1–4.

[11] S. Abadpour, A. Diewald, M. Pauli, and T. Zwick, “Extraction of scattering centers using a 77 ghz fmcw radar,” in 2019 12th German Microwave Conference (GeMiC), 2019, pp. 79–82.

[12] A. P. Sligar, “Machine learning-based radar perception for autonomous vehicles using full physics simulation,” IEEE Access, vol. 8, pp. 51 470–51 476, 2020.

[13] E. Schubert, M. Kunert, A. Frischen, and W. Menzel, “A multi-reflection-point target model for classification of pedestrians by automotive radar,” in 2014 11th European Radar Conference, 2014, pp. 181–184.

[14] P. Berthold, M. Michaelis, T. Luettel, D. Meissner, and H. Wuensche, “Radar reflection characteristics of vehicles for contour and feature estimation,” in 2017 Sensor Data Fusion: Trends, Solutions, Applications (SDF), 2017, pp. 1–6.

[15] M. Buhren and Bin Yang, “Simulation of automotive radar target lists using a novel approach of object representation,” in 2006 IEEE Intelligent Vehicles Symposium, 2006, pp. 314–319.

[16] P. Berthold, M. Michaelis, T. Luettel, D. Meissner, and H. Wuensche, “An abstracted radar measurement model for extended object tracking,” in 2018 21st International Conference on Intelligent Transportation Systems (ITSC), 2018, pp. 3866–3872.

[17] Y. Xia, P. Wang, K. Berntorp, T. Koike-Akino, H. Mansour, M. Pajovic, P. Boufounos, and P. Orlik, “Extended object tracking using hierarchical truncation measurement model with automotive radar,” 04 2020.

[18] E. Bel Kamel, A. Peden, and P. Pajusco, “Rcs modeling and measurements for automotive radar applications in the w band,” in 2017 11th European Conference on Antennas and Propagation (EUCAP), 2017, pp. 2445– 2449.

[19] P. Held, D. Steinhauser, A. Kamann, A. Koch, T. Brandmeier, and U. T. Schwarz, “Micro-doppler extraction of bicycle pedaling movements using automotive radar,” in 2019 IEEE Intelligent Vehicles Symposium (IV), 2019, pp. 744–749.

[20] M. Stolz, E. Schubert, F. Meinl, M. Kunert, and W. Menzel, “Multi-target reflection point model of cyclists for automotive radar,” in 2017 European Radar Conference (EURAD), 2017, pp. 94–97.

[21] “Inras products - radarbook,” 2020. [Online]. Available:

http://www.inras.at/en/products/radarbook.html

[22] J. Redmon and A. Farhadi, “YOLOv3: An Incremental Improvement,” arXiv e-prints, p. arXiv:1804.02767, Apr. 2018.

[23] R. Perez, F. Schubert, R. Rasshofer, and E. Biebl, ´ “Deep learning radar object detection and classification for urban automotive scenarios,” in 2019 Kleinheubach Conference, 2019, pp. 1–4.