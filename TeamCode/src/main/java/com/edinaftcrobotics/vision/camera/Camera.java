package com.edinaftcrobotics.vision.camera;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;

public class Camera {

    enum CameraType {
        Unknown, Front, Back, Web
    }

    protected VuforiaLocalizer.Parameters _params = new VuforiaLocalizer.Parameters();
    protected POCVuforia _pocVuforia = null;
    protected CameraType _cameratype;


    public Camera() {
        _params.cameraMonitorViewIdParent = R.id.cameraMonitorViewId;
        _params.vuforiaLicenseKey = "ASA9XvT/////AAABmUnq30r9sU3Nmf/+RS+Xx0CHgJj/JtD5ycahnuM/0B2SFvbMRPIZCbLi4LeOkfse9Dymor5W7vNMYI+vmqVx9kpEaKE8VM7cFMUb/T1LLwlCPdX9QKOruzTcRdlYswR7ULh4K11GuFZDO/45pSks+Nf25kT5cnV+IN3TsscA0o7I6XPIeUoAJJPsjw+AycsmRk2uffr3Bnupexr93iRfHylniqP+ss4cRcT1lOqS5Zhh7FQaoelR58qL/RUorGpknjy9ufCn9ervc6Mz01u3ZkM/EOa5wUPT8bDzPZ6nMDaadqumorT5Py+GtJSUosUgz4Gd3iR++fdEk6faFZq3L9xfBSagNykwhiyYx+oqwVqe";

        _cameratype = CameraType.Unknown;
    }


    public CameraType getcameratype(){
        return _cameratype;
    }

    public void activate() { _pocVuforia = new POCVuforia(_params); }

    public POCVuforia getPOCVuforia() { return _pocVuforia; }

    public  void deactivate() {
        _pocVuforia.closeAll();
        _pocVuforia = null;
    }
}
