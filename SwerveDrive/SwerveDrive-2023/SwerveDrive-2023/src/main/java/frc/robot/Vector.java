package frc.robot;

public final class Vector {
  public double x;
  public double y;

  public Vector(double dx, double dy) {
    x = dx;
    y = dy;
  }

  public double length() {
    return Math.sqrt(x * x + y * y);
  }

  public static double length(Vector a) {
    return Math.sqrt(a.x * a.x + a.y * a.y);
  }

  public Vector clone() {
    return new Vector(x, y);
  }

  public static Vector normalize(Vector a) {
    Vector temp = a.clone();
    temp.div(temp.length());
    return temp;
  }

  public void add(Vector e) {
    x += e.x;
    y += e.y;
  }

  public void sub(Vector e) {
    x -= e.x;
    y -= e.y;
  }

  public void mul(Vector e) {
    x *= e.x;
    y *= e.y;
  }

  public void div(Vector e) {
    x /= e.x;
    y /= e.y;
  }

  public void mul(double f) {
    x *= f;
    y *= f;
  }

  public void div(double f) {
    x /= f;
    y /= f;
  }
}
