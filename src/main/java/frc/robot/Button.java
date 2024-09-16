package frc.robot;

public enum Button
{
    B1(1),
    B2(2),
    B3(3),
    B4(4),
    LB(5),
    RB(6),
    LT(7),
    RT(8),
    Back(9),
    Start(10),
    LJ(11),
    RJ(12);

    private int port;

    Button(int port)
    {
        this.port = port;
    }
    public int getPort()
    {
        return port;
    }
}

