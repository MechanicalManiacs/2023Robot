package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Kalman;

import com.github.sh0nk.matplotlib4j.Plot;
import com.github.sh0nk.matplotlib4j.PythonExecutionException;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.StringTokenizer;

public class KalmanFilter265 {

    private static double start = System.currentTimeMillis();

    private static double xp = 0;
    private static double xv = 0;
    private static double xa = 0;

    private static double yp = 0;
    private static double yv = 0;
    private static double ya = 0;

    private static double dt = 1;
    private static double vara = 1.302215411599171091;

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

    private static Plot plt = Plot.create();

    private static File data = new File("C:\\Users\\Saranga\\AndroidStudioProjects\\OffSeasonUltimateGoal\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\fishlo\\v1\\fishlo\\robot\\Kalman\\data.txt");
    private static BufferedReader reader;
    private static StringTokenizer tokenizer;

    private static ArrayList<Double> x_list = new ArrayList<>();
    private static ArrayList<Double> y_list = new ArrayList<>();
    private static ArrayList<Double> xp_list = new ArrayList<>();
    private static ArrayList<Double> yp_list = new ArrayList<>();
    private static ArrayList<Double> xs_list = new ArrayList<>();
    private static ArrayList<Double> ys_list = new ArrayList<>();

    public static void main(String[] args) throws IOException, PythonExecutionException {
        //System State
        double[][] Xmat =  {{xp, xv, xa, yp, yv, ya}};
        X = new BlockRealMatrix(Xmat);

        //Estimate Uncertainty
        P = MatrixUtils.createRealIdentityMatrix(6);
        P.scalarMultiply(500);

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
        X.multiply(F);
        Ft = F.transpose();

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

        reader = new BufferedReader(new FileReader(data));
        for (int i = 0; i < 34; i++) {
            //mes eq
            tokenizer = new StringTokenizer(reader.readLine());
            double xm = Double.parseDouble(tokenizer.nextToken());
            double ym = Double.parseDouble(tokenizer.nextToken());
            xs_list.add(xm);
            ys_list.add(ym);
            Z = new Array2DRowRealMatrix(new double[] {xm, ym});

            double[][] Hmat =
                    {
                            {1, 0, 0, 0, 0, 0},
                            {0, 0, 0, 1, 0, 0}
                    };
            H = new Array2DRowRealMatrix(Hmat);
            Ht = H.transpose();

            //Measurement Uncertainty
            double varxm = 9;
            double varym = 9;

            R = new Array2DRowRealMatrix(new double[][] { {varxm, 0}, {0, varym} } );

            //Kalman Gain
            K1 = P.multiply(Ht);

            k_21 = H.multiply(P);

            K2 = k_21.multiply(Ht);

            K3 = K2.add(R);
            K4 = MatrixUtils.inverse(K3);

            K = K1.multiply(K4);

            //State Update
            XN1 = H.multiply(X);
            XN2 = Z.subtract(XN1);
            XN3 = K.multiply(XN2);
            XN = X.add(XN3);
            X = XN;
            System.out.println(X.getEntry(0, 0) + " " + X.getEntry(3, 0));
            x_list.add(X.getEntry(0, 0));
            y_list.add(X.getEntry(3, 0));

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
            System.out.println(X.getEntry(0, 0) + " " + X.getEntry(3, 0));

            plt.xlabel("object location on the x-axis");
            plt.ylabel("obkect location on the y-axis");
        }
        double end = System.currentTimeMillis();
        System.out.println("Runtime: " + (start - end));
        plt.plot().add(x_list).add(y_list).label("Kalman Filter Estimate");
        plt.plot().add(xp_list).add(yp_list).label("Prediction");
        plt.plot().add(xs_list).add(ys_list).label("Sensory Input");
        plt.legend();
        plt.title("Object location on the xy-plane");
        plt.show();
    }

    public static RealMatrix cov_ext(RealMatrix P) {
        RealMatrix P1 = P.multiply(F);
        RealMatrix P2 = P1.multiply(Ft);
        P = P2.add(Q);
        return P;
    }
}
