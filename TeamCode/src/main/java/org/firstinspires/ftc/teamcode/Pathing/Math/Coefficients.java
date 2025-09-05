package org.firstinspires.ftc.teamcode.Pathing.Math;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;

import java.util.ArrayList;
import java.util.List;

public class Coefficients extends Point implements Enums.Pathing {
    private List<Double> coefficients = new ArrayList<>();
    private Polynomial polynomialPolynomial = Polynomial.UNDEFINED;

    public Coefficients (List<Double> desiredCoefficient) {
        for (int i = 0; i < desiredCoefficient.size(); i++) {
            coefficients.add(desiredCoefficient.get(i));
        }

        if (coefficients.size() == 0) {
            polynomialPolynomial = Polynomial.UNDEFINED;
        } else {
            if (coefficients.size() == 1) { polynomialPolynomial = Polynomial.constant; }
            else if (coefficients.size() == 2) { polynomialPolynomial = Polynomial.linear; }
            else if (coefficients.size() == 3) { polynomialPolynomial = Polynomial.quadratic; }
            else if (coefficients.size() == 4) { polynomialPolynomial = Polynomial.cubic; }
            else if (coefficients.size() == 5) { polynomialPolynomial = Polynomial.quartic; }
            else if (coefficients.size() == 6) { polynomialPolynomial = Polynomial.quintic; }
            else polynomialPolynomial = Polynomial.MULTIPLE;
        }

        if (coefficients.get(0) == 0) { polynomialPolynomial = Polynomial.UNDEFINED; }
    }

    public Polynomial getPolynomialType() { return polynomialPolynomial; }

    public Double getCoefficient(int i) { return coefficients.get(i); }
}
