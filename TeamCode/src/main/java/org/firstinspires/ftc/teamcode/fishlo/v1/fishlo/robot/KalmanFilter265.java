package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.spartronics4915.lib.T265Camera;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.StringTokenizer;



public class KalmanFilter265 {

    private double start = System.currentTimeMillis();

    private double xp = 0;
    private double xv = 0;
    private double xa = 0;

    private double yp = 0;
    private double yv = 0;
    private double ya = 0;

    private double xm = 0;
    private double ym = 0;

    private static double dt = 1;
    private static double vara = 51.268323291305;

    private static RealMatrix X;
    private static RealMatrix P;
    private static RealMatrix F;
    private static RealMatrix q;
    private static RealMatrix Q;
    private static RealMatrix Ft;
    private static RealMatrix Z;
    private static RealMatrix H;
    private static RealMatrix Ht;
    private static RealMatrix R;
    private static RealMatrix K1;
    private static RealMatrix k_21;
    private static RealMatrix K2;
    private static RealMatrix K3;
    private static RealMatrix K4;
    private static RealMatrix K;
    private static RealMatrix XN1;
    private static RealMatrix XN2;
    private static RealMatrix XN3;
    private static RealMatrix XN;
    private static RealMatrix I;
    private static RealMatrix PN_1;
    private static RealMatrix PN2;
    private static RealMatrix PN3;
    private static RealMatrix PN4;
    private static RealMatrix PN5;
    private static RealMatrix PN6;
    private static RealMatrix PN7;
    private static RealMatrix PN8;
    private static RealMatrix PN;

    private static ArrayList<Double> x_list = new ArrayList<>();
    private static ArrayList<Double> y_list = new ArrayList<>();
    private static ArrayList<Double> xp_list = new ArrayList<>();
    private static ArrayList<Double> yp_list = new ArrayList<>();
    private static ArrayList<Double> xs_list = new ArrayList<>();
    private static ArrayList<Double> ys_list = new ArrayList<>();

    public KalmanFilter265(double xm, double ym) {
        this.xm = xm;
        this.ym = ym;
    }

    public void filterSetup() {
        //System State
        double[][] Xmat =
                {
                        {xp},
                        {xv},
                        {xa},
                        {yp},
                        {yv},
                        {ya}
                };
        X = new Array2DRowRealMatrix(Xmat);

        //Estimate Uncertainty
        P = MatrixUtils.createRealIdentityMatrix(6);
        P = P.scalarMultiply(100);

        //State Transition
        double[][] Fmat =
                {
                        {1, dt, 0.5 * Math.pow(dt, 2), 0, 0, 0},
                        {0, 1, dt, 0, 0, 0},
                        {0, 0, 1, 0, 0, 0},
                        {0, 0, 0, 1, dt, 0.5 * Math.pow(dt, 2)},
                        {0, 0, 0, 0, 1, dt},
                        {0, 0, 0, 0, 0, 1}
                };
        F = new Array2DRowRealMatrix(Fmat);

        //State Extrapolation
        X = F.multiply(X);
        Ft = F.transpose();
        System.out.println(X);

        //Noise
        double[][] Qmat =
                {
                        {0.25 * Math.pow(dt, 4), 0.5 * Math.pow(dt, 3), 0.5 * Math.pow(dt, 2), 0, 0, 0},
                        {0.5 * Math.pow(dt, 3), Math.pow(dt, 2), dt, 0, 0, 0},
                        {0.5 * Math.pow(dt, 2), dt, 1, 0, 0 ,0},
                        {0, 0, 0, 0.25 * Math.pow(dt, 4), 0.5 * Math.pow(dt, 3), 0.5 * Math.pow(dt, 2)},
                        {0, 0, 0, 0.5 * Math.pow(dt, 3), Math.pow(dt, 2), dt},
                        {0, 0, 0, 0.5 * Math.pow(dt, 2), dt, 1}
                };
        q = new Array2DRowRealMatrix(Qmat);
        Q = q.scalarMultiply(vara);

        //Covariance
        P = cov_ext(P);


    }

    public RealMatrix cov_ext(RealMatrix P) {
        RealMatrix P1 = F.multiply(P);
        RealMatrix P2 = P1.multiply(Ft);
        P = P2.add(Q);
        return P;
    }

    public void runFilter() {
        //mes eq
        double xm = this.xm;
        double ym = this.ym;
        xs_list.add(xm);
        ys_list.add(ym);
        double[][] xmymMat =
                {
                        {xm},
                        {ym}
                };
        Z = new Array2DRowRealMatrix(xmymMat);

        double[][] Hmat =
                {
                        {1, 0, 0, 0, 0, 0},
                        {0, 0, 0, 1, 0, 0}
                };
        H = new Array2DRowRealMatrix(Hmat);
        Ht = H.transpose();

        //Measurement Uncertainty
        double varxm = 144;
        double varym = 144;

        double[][] Rmat =
                {
                        {varxm,  0},
                        {0, varym}
                };
        R = new Array2DRowRealMatrix(Rmat);

        //Kalman Gain
        K1 = P.multiply(Ht);

        k_21 = H.multiply(P);

        K2 = k_21.multiply(Ht);

        K3 = K2.add(R);
        K4 = new LUDecomposition(K3).getSolver().getInverse();

        K = K1.multiply(K4);

        //State Update
        XN1 = H.multiply(X);
        XN2 = Z.subtract(XN1);
        XN3 = K.multiply(XN2);
        XN = X.add(XN3);
        X = XN;

        //Covariance Update
        I = MatrixUtils.createRealIdentityMatrix(6);
        PN_1 = K.multiply(H);
        PN2 = I.subtract(PN_1);

        //(I-KH)P(I-KH)^T+KRK^T
        PN3 = PN2.transpose();
        PN4 = PN2.multiply(P);
        PN5 = PN4.multiply(PN3);

        PN6 = K.transpose();
        PN7 = K.multiply(R);
        PN8 = PN7.multiply(PN6);

        PN = PN5.add(PN8);
        P = PN;

        //Predict
        X = F.multiply(X);
        P = cov_ext(P);
    }

    public void setxm(double xm) {
        this.xm = xm;
    }

    public void setym(double ym) {
        this.ym = ym;
    }

    public double getStateUpdateX() {
        return X.getEntry(0, 0);
    }

    public double getStateUpdateY() {
        return X.getEntry(3, 0);
    }

    public double getGain() {
        return K.getEntry(0,0);
    }
}
