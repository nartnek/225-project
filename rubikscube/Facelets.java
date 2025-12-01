package rubikscube;

public final class Facelets {

    private Facelets() {
    }

    public static final byte U1 = 0;
    public static final byte U2 = 1;
    public static final byte U3 = 2;
    public static final byte U4 = 3;
    public static final byte U5 = 4;
    public static final byte U6 = 5;
    public static final byte U7 = 6;
    public static final byte U8 = 7;
    public static final byte U9 = 8;

    public static final byte R1 = 9;
    public static final byte R2 = 10;
    public static final byte R3 = 11;
    public static final byte R4 = 12;
    public static final byte R5 = 13;
    public static final byte R6 = 14;
    public static final byte R7 = 15;
    public static final byte R8 = 16;
    public static final byte R9 = 17;

    public static final byte F1 = 18;
    public static final byte F2 = 19;
    public static final byte F3 = 20;
    public static final byte F4 = 21;
    public static final byte F5 = 22;
    public static final byte F6 = 23;
    public static final byte F7 = 24;
    public static final byte F8 = 25;
    public static final byte F9 = 26;

    public static final byte D1 = 27;
    public static final byte D2 = 28;
    public static final byte D3 = 29;
    public static final byte D4 = 30;
    public static final byte D5 = 31;
    public static final byte D6 = 32;
    public static final byte D7 = 33;
    public static final byte D8 = 34;
    public static final byte D9 = 35;

    public static final byte L1 = 36;
    public static final byte L2 = 37;
    public static final byte L3 = 38;
    public static final byte L4 = 39;
    public static final byte L5 = 40;
    public static final byte L6 = 41;
    public static final byte L7 = 42;
    public static final byte L8 = 43;
    public static final byte L9 = 44;

    public static final byte B1 = 45;
    public static final byte B2 = 46;
    public static final byte B3 = 47;
    public static final byte B4 = 48;
    public static final byte B5 = 49;
    public static final byte B6 = 50;
    public static final byte B7 = 51;
    public static final byte B8 = 52;
    public static final byte B9 = 53;


    public static final byte U = 0;
    public static final byte R = 1;
    public static final byte F = 2;
    public static final byte D = 3;
    public static final byte L = 4;
    public static final byte B = 5;

    public static final byte Ux1 = 0;
    public static final byte Ux2 = 1;
    public static final byte Ux3 = 2;

    public static final byte Rx1 = 3;
    public static final byte Rx2 = 4;
    public static final byte Rx3 = 5;

    public static final byte Fx1 = 6;
    public static final byte Fx2 = 7;
    public static final byte Fx3 = 8;

    public static final byte Dx1 = 9;
    public static final byte Dx2 = 10;
    public static final byte Dx3 = 11;

    public static final byte Lx1 = 12;
    public static final byte Lx2 = 13;
    public static final byte Lx3 = 14;

    public static final byte Bx1 = 15;
    public static final byte Bx2 = 16;
    public static final byte Bx3 = 17;


    public static final byte[][] CORNER_FACELETS;
    public static final byte[][] EDGE_FACELETS;

    static {
        CORNER_FACELETS = new byte[8][3];

        CORNER_FACELETS[0] = new byte[]{U9, R1, F3};
        CORNER_FACELETS[1] = new byte[]{U7, F1, L3};
        CORNER_FACELETS[2] = new byte[]{U1, L1, B3};
        CORNER_FACELETS[3] = new byte[]{U3, B1, R3};
        CORNER_FACELETS[4] = new byte[]{D3, F9, R7};
        CORNER_FACELETS[5] = new byte[]{D1, L9, F7};
        CORNER_FACELETS[6] = new byte[]{D7, B9, L7};
        CORNER_FACELETS[7] = new byte[]{D9, R9, B7};

        EDGE_FACELETS = new byte[12][2];

        EDGE_FACELETS[0]  = new byte[]{U6, R2};
        EDGE_FACELETS[1]  = new byte[]{U8, F2};
        EDGE_FACELETS[2]  = new byte[]{U4, L2};
        EDGE_FACELETS[3]  = new byte[]{U2, B2};
        EDGE_FACELETS[4]  = new byte[]{D6, R8};
        EDGE_FACELETS[5]  = new byte[]{D2, F8};
        EDGE_FACELETS[6]  = new byte[]{D4, L8};
        EDGE_FACELETS[7]  = new byte[]{D8, B8};
        EDGE_FACELETS[8]  = new byte[]{F6, R4};
        EDGE_FACELETS[9]  = new byte[]{F4, L6};
        EDGE_FACELETS[10] = new byte[]{B6, L4};
        EDGE_FACELETS[11] = new byte[]{B4, R6};
    }
    // -------- Move → string representation (optional) --------

    public static final String[] MOVE_STRINGS = 
            {"U", "UU", "UUU",
            "R ", "RR", "R'",
            "F ", "FF", "FFF",
            "D", "DD", "DDD",
            "L ", "LL", "LLL",
            "B", "BB", "BBB"};


    public static final int[][] CNK = buildCnkTable(13);

    private static int[][] buildCnkTable(int size) {
        int[][] cnk = new int[size][size];
        for (int n = 0; n < size; n++) {
            cnk[n][0] = 1;
            cnk[n][n] = 1;
            for (int k = 1; k < n; k++) {
                cnk[n][k] = cnk[n - 1][k - 1] + cnk[n - 1][k];
            }
        }
        return cnk;
    }

    public static final int[] UD_TO_STD = {
            Ux1, Ux2, Ux3,
            Rx2, Fx2,
            Dx1, Dx2, Dx3,
            Lx2, Bx2,
            Rx1, Rx3,
            Fx1, Fx3,
            Lx1, Lx3,
            Bx1, Bx3
    };

    public static final int[] STD_TO_UD = new int[18];

    public static final int[] CK_MV2BIT = new int[11];

    static {
        for (int udIndex = 0; udIndex < UD_TO_STD.length; udIndex++) {
            int std = UD_TO_STD[udIndex];
            STD_TO_UD[std] = udIndex;
        }


        for (int i = 0; i < 10; i++) {
            int stdI = UD_TO_STD[i];
            int axisI = stdI / 3;

            int mask = 0;
            for (int j = 0; j < 10; j++) {
                int stdJ = UD_TO_STD[j];
                int axisJ = stdJ / 3;

                boolean sameAxis = (axisI == axisJ);
                boolean sameLayer = (axisI % 3 == axisJ % 3) && (axisI >= axisJ);

                if (sameAxis || sameLayer) {
                    mask |= (1 << j);
                }
            }
            CK_MV2BIT[i] = mask;
        }
        CK_MV2BIT[10] = 0;
    }

    public static final long PERM_POOL_INIT;

    public static final long PERM_DECR_TEMPLATE;

    static {
        long pool = 0L;
        for (int nib = 15; nib >= 0; nib--) {
            pool = (pool << 4) | nib;
        }
        PERM_POOL_INIT = pool;

        long decr = 0L;
        for (int pos = 0; pos < 16; pos++) {
            decr |= (1L << (pos * 4));
        }
        decr &= ~0xFL; 
        PERM_DECR_TEMPLATE = decr;
    }
}
