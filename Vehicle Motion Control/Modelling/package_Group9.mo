within ;
package package_Group9

  model _1D_Task1
    // In this code we add a 1st order time delay of the torque generation of the
    // Prime Mover, a Drive Shafts Stiffness and a Tyre Slip phenomena.


    constant Real g=Modelica.Constants.g_n;

    //Driver and DriverInterpretor:
    Modelica.Units.SI.DimensionlessRatio APed;

    //VehCtrl:
    Modelica.Units.SI.Force F_xReq;
    Modelica.Units.SI.Torque T_PmReq1;

    //PrimeMover:
    parameter Modelica.Units.SI.Torque T_PmMax=200;
    parameter Modelica.Units.SI.Power P_PmMax=20e3;
    parameter Modelica.Units.SI.Inertia J_Pm=0.25;
    parameter Modelica.Units.SI.Time TimeConstant=0.05;
    Modelica.Units.SI.Torque T_PmReq2, T_PmInternal, T_Pm;
    Modelica.Units.SI.AngularVelocity w_Pm;

    //Gear:
    parameter Modelica.Units.SI.DimensionlessRatio Ratio=10;
    Modelica.Units.SI.Torque T_WhlRot;
    Modelica.Units.SI.AngularVelocity w_WhlRot;

    //Drivehaft
    parameter Modelica.Units.SI.RotationalSpringConstant C_Drv=23000;
    Modelica.Units.SI.AngularVelocity w_Drv;

    //Wheel and Tyre:
    parameter Modelica.Units.SI.Length Radius=0.3;
    parameter Modelica.Units.SI.Inertia J_Whl=0.5;
    parameter Modelica.Units.SI.DimensionlessRatio CC_x=20, mu_Peak=0.6;
    Modelica.Units.SI.Force F_xWhl, F_zWhl;
    Modelica.Units.SI.Velocity v_xWhl;
    Modelica.Units.SI.DimensionlessRatio s_x;

    //Vehicle:
    parameter Modelica.Units.SI.Mass m=1500;
    Modelica.Units.SI.Velocity v_x;

    //Model variant control:
    parameter Boolean ModelTyreSlip=true; // =true; //
    parameter Boolean ModelWheelInertia=false; // =true; //
    parameter Boolean ModelTyreFrictionSaturation=false; // =true; //
    parameter Modelica.Units.SI.Velocity v_Eps=0.001;

  initial equation
    v_x = 0;

  equation

    //Driver and DriverInterpretor (or LongCtrl):
    APed=if time<1 then 0 else 1;
    F_xReq=APed*m*g;

    //VehCtrl:
    T_PmReq1=F_xReq*Radius/Ratio;

    //PrimeMover:
    if w_Pm < P_PmMax/T_PmMax then
      T_PmReq2 = min(T_PmReq1,T_PmMax);
    else
      T_PmReq2 = min(T_PmReq1,P_PmMax/w_Pm);
    end if;
    der(T_PmInternal)=(T_PmReq2-T_PmInternal)/TimeConstant;
    J_Pm*der(w_Pm)=T_PmInternal-T_Pm;

    //Gear:
    T_WhlRot=T_Pm*Ratio;
    w_Drv=w_Pm/Ratio;

    //Driveshaft
    der(T_WhlRot)=C_Drv*(w_Drv-w_WhlRot);

    //Wheel and Tyre:
    if not ModelWheelInertia then
      F_xWhl = T_WhlRot/Radius;
    else
      J_Whl*der(w_WhlRot) = T_WhlRot - F_xWhl*Radius;
    end if;
    if not ModelTyreSlip then
      v_xWhl=w_WhlRot*Radius;
      s_x=0;
    elseif not ModelTyreFrictionSaturation then
      F_xWhl=CC_x*F_zWhl*s_x;
      s_x=(Radius*w_WhlRot-v_xWhl)/max(abs(Radius*w_WhlRot),v_Eps);
    else
      F_xWhl=sign(s_x)*min(CC_x*F_zWhl*abs(s_x), (mu_Peak-0.1*(abs(s_x)-mu_Peak/CC_x))*F_zWhl);
      s_x=min(Radius*w_WhlRot-v_xWhl)/max(abs(Radius*w_WhlRot),v_Eps);
    end if;

    //Vehicle Body:
    v_x=v_xWhl;
    m*der(v_x)=F_xWhl;
    m*g=F_zWhl;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(StopTime=5, __Dymola_Algorithm="Dassl"));
  end _1D_Task1;

  model compare_1_5D_Task1

    package_Group9._1D_Task1 refVeh
      annotation (Placement(transformation(extent={{-30,8},{-2,22}})));
    package_Group9._1D_Task1 New(C_Drv=30e3) "vehicle with lower gear ratio"
      annotation (Placement(transformation(extent={{8,-20},{36,-6}})));
    package_Group9._1D_Task1 New2(C_Drv=46e3) "vehicle with lower gear ratio"
      annotation (Placement(transformation(extent={{8,-20},{36,-6}})));
    package_Group9._1D_Task1 New3(C_Drv=60e3) "vehicle with lower gear ratio"
      annotation (Placement(transformation(extent={{8,-20},{36,-6}})));
    package_Group9._1D_Task1 New4(C_Drv=100e3) "vehicle with lower gear ratio"
      annotation (Placement(transformation(extent={{8,-20},{36,-6}})));
    package_Group9._1D_Task1 New5(C_Drv=110e3) "vehicle with lower gear ratio"
      annotation (Placement(transformation(extent={{8,-20},{36,-6}})));
    package_Group9._1D_Task1 New6( C_Drv=120e3) "vehicle with lower gear ratio"
      annotation (Placement(transformation(extent={{8,-20},{36,-6}})));
    parameter Modelica.Units.SI.Torque T_PmMax=300;
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
      experiment(StopTime=10, __Dymola_Algorithm="Dassl"));
  end compare_1_5D_Task1;

  model _1D_Task2_1

    // In this code we add a Longitudinal Load Transfer to the model presented
    // in the Task1 using a rigid suspension model.

    constant Real g=Modelica.Constants.g_n;

    //Driver and DriverInterpretor:
    Modelica.Units.SI.DimensionlessRatio APed;

    //VehCtrl:
    Modelica.Units.SI.Force F_xReq;
    Modelica.Units.SI.Torque T_PmReq1;

    //PrimeMover:
    parameter Modelica.Units.SI.Torque T_PmMax=200;
    parameter Modelica.Units.SI.Power P_PmMax=20e3;
    parameter Modelica.Units.SI.Inertia J_Pm=0.25;
    parameter Modelica.Units.SI.Time TimeConstant=0.05;
    Modelica.Units.SI.Torque T_PmReq2, T_PmInternal, T_Pm;
    Modelica.Units.SI.AngularVelocity w_Pm;

    //Gear:
    parameter Modelica.Units.SI.DimensionlessRatio Ratio=10;
    Modelica.Units.SI.Torque T_WhlRot;
    Modelica.Units.SI.AngularVelocity w_WhlRot;

    //Drivehaft
    parameter Modelica.Units.SI.RotationalSpringConstant C_Drv=23000;
    Modelica.Units.SI.AngularVelocity w_Drv;

    //Suspension
    parameter Modelica.Units.SI.TranslationalSpringConstant C_z=30e3;
    parameter Modelica.Units.SI.DampingCoefficient D_z=2.5e3;
    parameter Modelica.Units.SI.Inertia J_y=3000;

    //Wheel and Tyre:
    parameter Modelica.Units.SI.Length Radius=0.3;
    parameter Modelica.Units.SI.Inertia J_Whl=0.5;
    parameter Modelica.Units.SI.DimensionlessRatio CC_x=20, mu_Peak=0.6;
    Modelica.Units.SI.Force F_xWhl, F_zWhl;
    Modelica.Units.SI.Velocity v_xWhl;
    Modelica.Units.SI.DimensionlessRatio s_x;

    //Vehicle:
    parameter Modelica.Units.SI.Mass m=1500;
    parameter Modelica.Units.SI.Length lf=1.25;
    parameter Modelica.Units.SI.Length lr=1.5;
    parameter Modelica.Units.SI.Length h=0.4;
    Modelica.Units.SI.Velocity v_x;
    Modelica.Units.SI.Force F_rz;

    //Model variant control:
    parameter Boolean ModelTyreSlip=true; // =true; //
    parameter Boolean ModelWheelInertia=false; // =true; //
    parameter Boolean ModelTyreFrictionSaturation=false; // =true; //
    parameter Modelica.Units.SI.Velocity v_Eps=0.001;

  initial equation
    v_x = 0;

  equation

    //Driver and DriverInterpretor (or LongCtrl):
    APed=if time<1 then 0 else 1;
    F_xReq=APed*m*g;

    //VehCtrl:
    T_PmReq1=F_xReq*Radius/Ratio;

    //PrimeMover:
    if w_Pm < P_PmMax/T_PmMax then
      T_PmReq2 = min(T_PmReq1,T_PmMax);
    else
      T_PmReq2 = min(T_PmReq1,P_PmMax/w_Pm);
    end if;
    der(T_PmInternal)=(T_PmReq2-T_PmInternal)/TimeConstant;
    J_Pm*der(w_Pm)=T_PmInternal-T_Pm;

    //Gear:
    T_WhlRot=T_Pm*Ratio;
    w_Drv=w_Pm/Ratio;

    //Driveshaft
    der(T_WhlRot)=C_Drv*(w_Drv-w_WhlRot);

    //Wheel and Tyre:
    if not ModelWheelInertia then
      F_xWhl = T_WhlRot/Radius;
    else
      J_Whl*der(w_WhlRot) = T_WhlRot - F_xWhl*Radius;
    end if;
    if not ModelTyreSlip then
      v_xWhl=w_WhlRot*Radius;
      s_x=0;
    elseif not ModelTyreFrictionSaturation then
      F_xWhl=CC_x*F_zWhl*s_x;
      s_x=(Radius*w_WhlRot-v_xWhl)/max(abs(Radius*w_WhlRot),v_Eps);
    else
      F_xWhl=sign(s_x)*min(CC_x*F_zWhl*abs(s_x), (mu_Peak-0.1*(abs(s_x)-mu_Peak/CC_x))*F_zWhl);
      s_x=min(Radius*w_WhlRot-v_xWhl)/max(abs(Radius*w_WhlRot),v_Eps);
    end if;

    //Vehicle Body:
    v_x=v_xWhl;
    m*der(v_x)=F_xWhl;
    m*g=F_zWhl+F_rz;
    F_rz*lr=F_zWhl*lf+F_xWhl*h;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(StopTime=5, __Dymola_Algorithm="Dassl"));
  end _1D_Task2_1;

  model _1D_Task2_2

    // In this code we add a Trivial Suspension compliance and damping to the
    // model presented in the Task2.1 so with the Load Transfer.

    constant Real g=Modelica.Constants.g_n;

    //Driver and DriverInterpretor:
    Modelica.Units.SI.DimensionlessRatio APed;

    //VehCtrl:
    Modelica.Units.SI.Force F_xReq;
    Modelica.Units.SI.Torque T_PmReq1;

    //PrimeMover:
    parameter Modelica.Units.SI.Torque T_PmMax=200;
    parameter Modelica.Units.SI.Power P_PmMax=20e3;
    parameter Modelica.Units.SI.Inertia J_Pm=0.25;
    parameter Modelica.Units.SI.Time TimeConstant=0.05;
    Modelica.Units.SI.Torque T_PmReq2, T_PmInternal, T_Pm;
    Modelica.Units.SI.AngularVelocity w_Pm;

    //Gear:
    parameter Modelica.Units.SI.DimensionlessRatio Ratio=10;
    Modelica.Units.SI.Torque T_WhlRot;
    Modelica.Units.SI.AngularVelocity w_WhlRot;

    //Drivehaft
    parameter Modelica.Units.SI.RotationalSpringConstant C_Drv=23000;
    Modelica.Units.SI.AngularVelocity w_Drv;

    //Suspension
    parameter Modelica.Units.SI.TranslationalSpringConstant C_z=30e3;
    parameter Modelica.Units.SI.TranslationalDampingConstant D_z=2.5e3;
    parameter Modelica.Units.SI.Inertia J_y=3000;
    Modelica.Units.SI.AngularVelocity w_b;
    Modelica.Units.SI.Velocity v_zr;
    Modelica.Units.SI.Velocity v_zf;
    Modelica.Units.SI.Force F_sr;
    Modelica.Units.SI.Force F_dr;
    Modelica.Units.SI.Force F_sf;
    Modelica.Units.SI.Force F_df;

    //Wheel and Tyre:
    parameter Modelica.Units.SI.Length Radius=0.3;
    parameter Modelica.Units.SI.Inertia J_Whl=0.5;
    parameter Modelica.Units.SI.DimensionlessRatio CC_x=20, mu_Peak=0.6;
    Modelica.Units.SI.Force F_xWhl, F_zWhl;
    Modelica.Units.SI.Velocity v_xWhl;
    Modelica.Units.SI.DimensionlessRatio s_x;

    //Vehicle:
    parameter Modelica.Units.SI.Mass m=1500;
    parameter Modelica.Units.SI.Length lf=1.25;
    parameter Modelica.Units.SI.Length lr=1.5;
    parameter Modelica.Units.SI.Length h=0.4;
    Modelica.Units.SI.Velocity v_x;
    Modelica.Units.SI.Force F_rz;

    //Model variant control:
    parameter Boolean ModelTyreSlip=true; // =true; //
    parameter Boolean ModelWheelInertia=false; // =true; //
    parameter Boolean ModelTyreFrictionSaturation=false; // =true; //
    parameter Modelica.Units.SI.Velocity v_Eps=0.001;

   // Modelica.Units.SI.DimensionlessRatio fn;

  initial equation
    v_x = 0;
    der(v_zf)=0;

  equation

    //Driver and DriverInterpretor (or LongCtrl):
    APed=if time<1 then 0 else 1;
    F_xReq=APed*m*g;

    //VehCtrl:
    T_PmReq1=F_xReq*Radius/Ratio;

    //PrimeMover:
    if w_Pm < P_PmMax/T_PmMax then
      T_PmReq2 = min(T_PmReq1,T_PmMax);
    else
      T_PmReq2 = min(T_PmReq1,P_PmMax/w_Pm);
    end if;
    der(T_PmInternal)=(T_PmReq2-T_PmInternal)/TimeConstant;
    J_Pm*der(w_Pm)=T_PmInternal-T_Pm;

    //Gear:
    T_WhlRot=T_Pm*Ratio;
    w_Drv=w_Pm/Ratio;

    //Driveshaft
    der(T_WhlRot)=C_Drv*(w_Drv-w_WhlRot);

    //Suspension
    der(F_sr)=C_z*v_zr;
    F_dr=D_z*v_zr;
    der(F_sf)=C_z*v_zf;
    F_df=D_z*v_zf;
    J_y*der(w_b)=-(F_sr+F_dr)*(lf+lr)-m*der(v_x)*h-m*g*lf;
    v_zf=-w_b*lf;
    v_zr=w_b*lr;

    //Wheel and Tyre:
    if not ModelWheelInertia then
      F_xWhl = T_WhlRot/Radius;
    else
      J_Whl*der(w_WhlRot) = T_WhlRot - F_xWhl*Radius;
    end if;
    if not ModelTyreSlip then
      v_xWhl=w_WhlRot*Radius;
      s_x=0;
    elseif not ModelTyreFrictionSaturation then
      F_xWhl=CC_x*F_zWhl*s_x;
      s_x=(Radius*w_WhlRot-v_xWhl)/max(abs(Radius*w_WhlRot),v_Eps);
    else
      F_xWhl=sign(s_x)*min(CC_x*F_zWhl*abs(s_x), (mu_Peak-0.1*(abs(s_x)-mu_Peak/CC_x))*F_zWhl);
      s_x=min(Radius*w_WhlRot-v_xWhl)/max(abs(Radius*w_WhlRot),v_Eps);
    end if;

    //Vehicle Body:
    v_x=v_xWhl;
    m*der(v_x)=F_xWhl;
    m*g=F_zWhl+F_rz;
    F_rz*lr=F_zWhl*lf+F_xWhl*h+J_y*der(w_b);

    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(StopTime=5, __Dymola_Algorithm="Dassl"));
  end _1D_Task2_2;
  annotation (uses(Modelica(version="4.0.0")));
end package_Group9;
