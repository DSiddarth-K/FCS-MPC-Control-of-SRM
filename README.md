# Single Finite Control Set MPC Control for an SRM With Asymmetric NPC 3-level Converter to Minimize Current Ripple

**Siddarth Kachu**

## Abstract

This paper covers the design of an MPC for an asymmetric NPC to control an SRM motor. The MPC was built and simulated in Simulink/Matlab. The model involves a discrete model of the SRM and the NPC inverter to calculate the next step current based on possible switching states. The best switching state is selected based on the cost function minimization and the weights of the terms. This switching state is applied to the Asymmetric NPC and the current is then calculated with a model of the SRM. The current profile from the controller is compared to the standard hysteresis controller for SRMs, and a PWM method proposed in [3]. It is found that while the MPC performs better than the hysteresis controller, the current ripple is not less than the PWM method for the same switching frequency, but the controller still shows promise for specific applications due to the freedom for the designer to select switching states based on various parameters.

# I. INTRODUCTION

## A. SRM Motor and Converter

With the increasing demand for electrification and a supply chain not prepared for the demand, SRMs have emerged as a potential replacement for PMSMs. These motors have no permanent magnets and are only made of electrical steel and copper. The stator of the motor has poles with copper windings and the rotor, only steel poles. When a pole is energized, it generates a magnetic field that magnetizes a rotor pole, attracting the pole to the coil [1]. This process is repeated for all the poles on the stator causing the rotor to spin. This motor has phases that are isolated from each other, the number of phases being dependent on the pole configuration [1]. Each phase is controlled with an asymmetric bridge leg that energizes the coil with switches or freewheels to allow for the current to drop and has diodes to carry current back to the source [1].

<div align="center"><img width="232" height="175" alt="image" src="https://github.com/user-attachments/assets/f638ffb7-d36c-4c92-886c-0293be4d1d1b"/>

  **Fig 1.A.1. Single phase of asymmetric bridge converter [1]**</div>
  
This converter allows for control of flux as it is proportional to the number of turns multiplied by the current squared, which is directly proportional to torque [1].

## B. NPC

The neutral point clamped inverter is a multilevel inverter topology that uses midpoints in the DC link capacitor bank along with clamping diode to apply additional voltage levels across the load. This topology allows for the use of lower rated components since each switch will only need to be rated for Vd/2 if there is one neutral point. This topology also provides good THD performance due to being capable of applying Vd, Vd/2, 0, -Vd/2, -Vd for the topology in Fig.1.B.1. The number of levels can be increased based on the number of neutral points, but this increases the difficulty in balancing the capacitor.

<div align="center"><img width="312" height="310" alt="image" src="https://github.com/user-attachments/assets/1b846a0f-98c9-419b-a62f-97212e274724" />

  **Fig 1.B.1 Single phase leg of NPC inverter [2]**</div>
## C. MPC

Model predictive control allows for the control of an inverter using a model of the inverter. The current time step current output measurements are input into a discrete model of the inverter, which outputs the next time step current into a cost function [2]. A current reference is extrapolated into the next time step current reference and input into the cost function; The cost function takes all the possible current values for the various switch configurations and determines what the best possible switching state is and applies it [2]. The best switching state is determined by using a minimizing equation with the desired parameters. This method, for example, can be used to balance capacitor charge in an NPC or NNPC inverter by weighing the capacitor voltages higher than the current.

# II. APPROACH

## A. SRM Model

To model this controller, a static analysis on an SRM motor must be completed to collect data to use in the modelling experiment. Motor geometry is created based on the paper [3] with the available information. Geometry in Fig. 2.1

<div align="center"><img width="265" height="262" alt="image" src="https://github.com/user-attachments/assets/5aaf3c28-d5d8-4244-82bb-c1e3627f5349" />

**Fig 2.A.1. 12/8 SRM model created in JMAG [2]**</div>

This motor geometry has 3-phase parallel coil configuration with 4 coils per phase. The A-phase is energized with 0 to 46 A in 2 A increment steps to collect a robust data set to run the dynamic analysis with. The experiment provides torque, flux linkage and induced voltage characteristics of the motor. These values are used to create lookup tables for the SRM model in MATLAB.

The modelling of this controller involves 3 models. The first model is a model of the SRM. The SRM model is based off the lecture notes in [1].

<div align="center"><img width="1301" height="544" alt="image" src="https://github.com/user-attachments/assets/00f1eef2-c446-4119-a6d6-b7ed8b213b80" />

**Fig 2.A.2. Electrical Angle of 3 SRM phases**</div>

First the electrical angle is calculated based on the 2000RPM operation speed specified in [3]. This outputs the electrical angles of each phase.

<div align="center"><img width="1151" height="620" alt="image" src="https://github.com/user-attachments/assets/c4901efd-04fc-4ea4-90c6-cf24cf4f0c71" />

**Fig 2.A.3. Excitation signal generator**</div>

Based on the electrical angle, the turn on/off angles excitation signals are generated based on the lecture notes in [1].

<div align="center"><img width="1006" height="241" alt="image" src="https://github.com/user-attachments/assets/257fa383-32a6-4880-8c66-3dbabf6b066c" />

**Fig 2.A.4. Phase Current Calculation**</div>

The final aspect of the SRM model is the current calculation. The difference between the measured phase voltage and the phase current times the phase resistance is calculated. The phase current is calculated by using a lookup table with the electrical angle and flux linkage breakpoints. The result is the phase flux linkage that builds up and decays with electrical angle. The phase currents from the lookup table are also sampled (at 50Khz in this case) and input into the MPC model.

## B. MPC Model

The asymmetric NPC discrete model takes the current time step phase current which is output from the SRM model. The phase flux linkage, electrical angle and excitation signal are also input into the block. The model generates a vector of the possible next step phase voltages as listed in the Uoutput column of Table 2.B.1.

### Table 2.B.1. Switching states of 5 Level Asymmetric NPC
<div align="center">
  
| State | S1 | S2 | S3 | S4 | Uoutput |
| ----- | -- | -- | -- | -- | ------- |
| 1     | 1  | 1  | 1  | 1  | Vdc     |
| 2     | 1  | 1  | 1  | 0  | 0.5Vdc  |
| 3     | 1  | 1  | 0  | 0  | 0       |
| 4     | 0  | 1  | 1  | 1  | 0.5Vdc  |
| 5     | 0  | 1  | 1  | 0  | 0       |
| 6     | 0  | 1  | 0  | 0  | -0.5Vdc |
| 7     | 0  | 0  | 1  | 1  | 0       |
| 8     | 0  | 0  | 1  | 0  | -0.5Vdc |
| 9     | 0  | 0  | 0  | 0  | -Vdc    |

</div>
There are 2 possible states for 0.5Vdc, -0.5Vdc and 3 possible states for applying 0 V, these redundant states allow for better control of the charge of the capacitors as they can either charge or discharge the DC link capacitors based on the direction of the current. For example, from table 2.B.1 state 2 will result in the midpoint increasing in charge as current is flowing into the midpoint, but state 4 results in a discharge of the capacitor as the current is flowing out of the midpoint. Although the redundant states are not used in this experiment, they will be accounted for in the vector for possible future implementations.
<br>
<div align="center">
  <img width="569" height="663" alt="image" src="https://github.com/user-attachments/assets/80403e9c-614a-437e-81a9-7acdac3a23ae" />

**Fig. 2.B.2. Next step Electrical angle and Flux Linkage**</div>

The next step electrical angle is calculated using the following equation:
<div align="center">
  
  $\phi(k+n) = \phi(k) + n\omega T = \phi(k) + n\left(N_{\mathrm{rpm}}\cdot\frac{360}{60}\right)T$
  
</div>

Where n, the size of the step, which will allow for the calculation to account for different decision intervals chosen. Since the possible voltages are stored in a vector of 9, the electrical angles are also stored in a vector of size 9.

The next step flux linkage is calculated using the following equation:
<div align="center">
  
$\lambda(k+n)=\lambda(k)+nT(V-IR)$

</div>
These next step values are input into the current lookup table from Fig 2.A.4 to get a vector of 9 possible I(K+n) values based on the possible switching configurations.

<div align="center">
<img width="824" height="106" alt="image" src="https://github.com/user-attachments/assets/6b350386-96d3-4c5e-8bf0-b601c9d25a46" />

**Fig. 2.B.3. Using look up table to calculate I(K+n)**

</div>
This vector is output into the cost function to determine the best possible switch stare based on the set weightings from the current minimization and applies the associated switching states.

<div align="center">
  
<img width="1485" height="547" alt="image" src="https://github.com/user-attachments/assets/8ee1183b-37a4-4324-b29a-4091c273703e" />

**Fig. 2.B.4. Cost function calculation block**
</div>
The 9 possible currents are subtracted from the reference current and squared. The smallest of the differences are indexed and selected by taking the largest index, since there are no other costs being considered for this experiment. Based on the vector index, the associated switching state is applied. This switching state is only applied when the excitation signal is high. When the excitation signal is low, the phase needs to be de-energized, so a negative voltage needs to be applied to drive current to 0, then a 0 V switching state needs to be applied. A new switching state is only applied after a certain number of samples; this is synced with the electrical angle and flux calculations. This allows for control of switching frequency for a reasonable comparison of performance. The cost function block outputs the states of the switches.

## C. Asymmetric 3-Level NPC

To properly control an SRM with an NPC converter, the proposed Asymmetric 3-Level NPC topology (Fig. 2.C.1) in [3] is used in this model.

<div align="center">
<img width="529" height="796" alt="image" src="https://github.com/user-attachments/assets/ad726f0b-faf1-4214-a0e2-8e333eea3aa4" />

**Fig. 2.C.1. Single phase leg of Asymmetric 3-Level NPC**
</div>

This topology adds freewheeling diodes to the NPC which are necessary to apply 0 V across the phase and for proper current regulation. There is a current source to simulate the coil of the SRM, controlled by the output of the “Phase Current Calculation” block in the SRM model. There is a voltage sensor measuring the phase voltage which is the input of the “Phase Current Calculation.” The DC-link capacitors are modelled with voltage sources, as capacitor balancing is not the focus of this experiment.

In addition to the 3 segments of the model, there is a simple fourth-order Lagrange extrapolation block to extrapolate the current reference. There is logic to increase the current reference to test the transient performance.

# III. ALTERNATIVE CONTROL METHODS

SRMs use a basic hysteresis controller with an asymmetric bridge converter, this will be used as a benchmark for the performance of the MPC controller. Hysteresis controllers apply Vdc to increase the current and -Vdc or 0 to decrease the current and maintain it within a set band. If the band is too small, it will increase the switching frequency, as the current will hit the band limit more often and need to switch, which will increase the losses of the inverter. Too large of a band will make the current ripple very large and reduce the torque quality. The same motor static characteristics will be applied to the hysteresis controller based on [1]. The current profiles from this controller will be compared to those of the MPC. Since the hysteresis controller only applies 3 levels a clear improvement is expected with the 5 levels available to the MPC controller with asymmetric NPC.

The MPC controller can also be compared to the proposed PWM modulation scheme for the asymmetric NPC converter in [3]. This method works by using 4 triangular carrier waves between each of the 5 voltage levels of the asymmetric NPC. The voltage that needs to be applied to the phases will be compared to the carrier waveforms. The inverter will apply the voltage level above and below the triangular wave based on if the reference voltage is higher or lower than the carrier [3]. This method resulted in a peak-to-peak current ripple of only 3A, with a switching frequency of 10Khz. This is the target to compare the proposed controller with.

# IV. SIMULATION RESULTS

## A. Hysteresis Controller

The following results are the output of the hysteresis controller designed in [1]. With a 2% hysteresis band, using soft switching scheme. Hard switching resulted in higher steady state ripple, worse transient ripples and much higher switching frequency.
<div align="center">
<img width="1363" height="660" alt="image" src="https://github.com/user-attachments/assets/f7215fb7-5d7f-40c2-b863-1e54552733ba" />

**Fig 3.A.1. Phase Voltage and Current Overlayed at 20A** </div>
<div align="center">
<img width="1361" height="666" alt="image" src="https://github.com/user-attachments/assets/6cf52008-963d-4400-abaf-02a0c7cc7010" />

**Fig 3.A.2. Current Ripple at 20A**</div>

With the hysteresis controller, there are large current spikes during the initial magnetization phase where the phase voltage is applied. There is a max current ripple of around 8A, with the soft switching controller with a reference current of 20A.

<div align="center"><img width="1356" height="662" alt="image" src="https://github.com/user-attachments/assets/4e7478ab-83ea-4f87-b6dc-9795ebeabc8a" />

**Fig 3.A.3. Phase Voltage and Current Overlayed at 40A**</div>

<div align="center"><img width="1362" height="668" alt="image" src="https://github.com/user-attachments/assets/0b9c5d33-6e7e-4f35-9dab-8974fc4ac926" />

**Fig 3.A.4. Current Ripple at 40A**</div>

With a reference current of 40A, there is a max ripple of around 8A. There is a smaller overshoot from the initial magnetization but the controller struggles to maintain the reference of 40A and overshoots often.

<div align="center"><img width="1366" height="669" alt="image" src="https://github.com/user-attachments/assets/b1894fad-6681-4b77-b4ba-0f55c82fd771" />

**Fig 3.A.5. Current Transient**</div>

The controller also has a significant overshoot during transients.

## B. MPC Controller

The MPC controller decides and selects a new switching configuration every sample period. This can make the switching frequency very high and impractical, the results for when the decision is limited to every 10Khz, 50Khz and no limit are collected.

<div align="center"><img width="1661" height="814" alt="image" src="https://github.com/user-attachments/assets/f72c41d2-d72f-4ce8-ae4c-2420e173d71b" />

**Fig 3.B.1. Current Ripple at 20A and 40A at 10Khz**</div>

Due to the controller making decisions and applying the switch configuration at 10Khz, the current waveform has very high ripple at both current levels and transients. There is a max ripple of 11A and 16A at 20 and 40A respectively. The initial magnetization at 40A was also difficult to regulate when the controller was forced to this low switching speed.

<div align="center"><img width="1652" height="803" alt="image" src="https://github.com/user-attachments/assets/d4728628-82f7-4034-8d24-5dda477a13a0" />

**Fig 3.B.2. Phase Voltage and Current Overlayed at 20A, 50Khz**</div>

When the controller was forced to switch at 50Khz, the waveforms smoothed, and current looked much more stable.

<div align="center"><img width="1663" height="807" alt="image" src="https://github.com/user-attachments/assets/da3dd1c1-3078-4e99-884f-49559b2c363a" />

**Fig 3.B.3. Current Ripple at 20A, 50Khz**</div>

There is no overshoot with this controller at 20A, but it does not reach the reference current value in the initial magnetization. The max ripple was 3.5A.

<div align="center"><img width="1649" height="801" alt="image" src="https://github.com/user-attachments/assets/74df5523-b0ea-48bd-ac7f-e8eb416e9585" />

**Fig 3.B.4. Phase Voltage and Current Overlayed at 40A, 50Khz**</div>

At 40A, the controller switched more often than at 20A.

<div align="center"><img width="1665" height="815" alt="image" src="https://github.com/user-attachments/assets/ec5bc07f-35b1-44f2-b61b-4ec5a11415e8" />

**Fig 3.B.5. Current Ripple at 40A, 50Khz**</div>

The initial magnetization does not result in the current meeting the reference value, the peak ripple was 8A.

<div align="center"><img width="1373" height="676" alt="image" src="https://github.com/user-attachments/assets/d88f66ab-a74a-4af5-969d-4093954d5ca9" />
 
**Fig 3.B.6. Current Transient, 50Khz**</div>

The current transient does not result in any overshoots.

<div align="center"><img width="1362" height="660" alt="image" src="https://github.com/user-attachments/assets/b0499cd9-9574-4197-9992-9f2e77b5a426" />

**Fig 3.B.7. Phase Voltage and Current Overlayed at 40A**</div>

Where the controller is not limited to a certain switching/decision frequency, it has a very high switching frequency.

<div align="center"><img width="1376" height="673" alt="image" src="https://github.com/user-attachments/assets/8acd68d3-c61c-425f-866c-426248afa512" />

**Fig 3.B.8. Current Ripple at 20A**</div>

The max current ripple at 20A is less than 0.5A, there is very little ripple at all.

<div align="center"><img width="1368" height="675" alt="image" src="https://github.com/user-attachments/assets/01838a9b-1961-40bb-9b41-2468f5597e3a" />

**Fig 3.B.9. Current Ripple at 40A**</div>

At 40A, there is a max current ripple of 0.5A.

<div align="center"><img width="1372" height="677" alt="image" src="https://github.com/user-attachments/assets/2f081388-4978-4da9-938b-159e9277ddd5" />

**Fig 3.B.10. Current Transients**</div>

The transient current waveforms are also very smooth and have a ripple of less than 1A.

# V. DISCUSSION

The MPC can perform very well, it performs better than the hysteresis controller when not limited to lower switching frequencies. Since motor properties such as physical design and inductance vary, this will determine how quickly the current decays, so the switching frequency can vary with the motor. For a 2% hysteresis band, it was noted that the hysteresis controller has a switching frequency of around 50Khz, for the SRM motor at 2000rpm. When compared to the MPC limited to 50Khz, it outperformed the hysteresis controller with less max ripple at both current values. In addition to the controller, having access to the 5 voltage levels of the NPC over the 3 of the asymmetric bridge was extremely beneficial to the ripple. Since there were 5 levels, the controller often chose to use Vdc and -Vdc levels to magnetize and demagnetize quicker while using the 0.5Vdc, 0, -0.5Vdc to regulate the current at the reference, with less ripple since there the current will increase slower with a lower voltage.

Although, The MPC could not maintain the current with less ripple than the proposed modulation scheme presented in [3], as there was a peak-to-peak ripple of 3A at a switching frequency of 10Khz. The MPC was only able to achieve a ripple that was low when the switching frequency was limited to 50Khz. The MPC can definitely perform better than the proposed modulation scheme when no limit is set, but this is not a fair comparison as a switching frequency of up to 500khz+ is not practical. It would be possible to add a cost function where there is no limit set, but switching to a different state is weighed as a method to discourage switching often. This would decrease the switching frequency and reduce losses while not limiting the controller to switching and making a decision at a set frequency.

# VI. CONCLUSION

Based on the results found, the MPC was not as effective as the proposed PWM method alongside the asymmetric NPC converter, although, it still has performance improvements over the basic hysteresis controller.

First, it has significantly better performance than the basic hysteresis controller but requires 2 more switches, and 2 more didoes per phase. The additional 2 levels provided with these devices pay off in the current response and should be used if the torque quality is critical. Additionally, MPC control is much better when switching around the same frequency.

The MPC control did not have lower ripple than the PWM control method with the same converter but can still have an application specific use case. The PWM controller is simple and can operate at a very low fixed frequency which might be desired in cases where efficiency is critical as higher frequency switching will generate losses and heat. When, higher frequency switching is possible, MPC can be chosen as it performs better, with the benefits of the cost function. In this experiment the cost function only consists of one term, the current. This cost function applies the voltage that minimizes the current in the next step, but this means that the cost function aspect of the MPC was underutilized. The MPC can be designed to focus on a specific application easily. For example, the cost function could include the state of the capacitor charge which was excluded in this experiment. This would allow for the controller to pick the redundant states that best balance the capacitor, without SVM. Another possible function could be to further reduce torque ripple by implementing torque sharing functions and weighing the minimization between the torque and the torque profile from the next step. The controller could also be expanded to 2 steps, for better control as this accounts for the states of the SRM and inverter for the next two time steps and provide more options but will get computationally expensive as this requires calculating the currents for the next two time steps for all possible phase voltages applied over those two steps. The controller could also be modified to choose the same switching state more often to naturally lower the switching frequency or favors a state that balances the load on the switches. For example, going from state 5 to 6 is the same as going from 5 to 8, but these states can be alternated to balance the load between both the switches. To conclude, the MPC is more complex and computationally expensive but can be designed for specific applications and can perform better than a hysteresis controller or make up for the loss in performance relative to the PWM controller with the versatility in design.

# REFERENCES

[1] B Bilgin. (2025). ECE 716 Switched Reluctance Machines – Converters in Switched Reluctance Machines [PowerPoint slides]

[2] M. Narimani and B. Wu. (2025). Topic 8 – Other Multilevel Inverter [PowerPoint slides]

[3] F. Peng, J. Ye and A. Emadi, "An Asymmetric Three-Level Neutral Point Diode Clamped Converter for Switched Reluctance Motor Drives," in IEEE Transactions on Power Electronics, vol. 32, no. 11, pp. 8618-8631, Nov. 2017, doi: 10.1109/TPEL.2016.2642339.
