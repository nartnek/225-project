package rubikscube;

import java.util.Arrays;

class CubeMapping {
    //Moves
    static final byte Ux1 = 0;
    static final byte Ux2 = 1;
    static final byte Ux3 = 2;
    static final byte Rx1 = 3;
    static final byte Rx2 = 4;
    static final byte Rx3 = 5;
    static final byte Fx1 = 6;
    static final byte Fx2 = 7;
    static final byte Fx3 = 8;
    static final byte Dx1 = 9;
    static final byte Dx2 = 10;
    static final byte Dx3 = 11;
    static final byte Lx1 = 12;
    static final byte Lx2 = 13;
    static final byte Lx3 = 14;
    static final byte Bx1 = 15;
    static final byte Bx2 = 16;
    static final byte Bx3 = 17;

    //Facelets
    static final byte U1 = 0;
    static final byte U2 = 1;
    static final byte U3 = 2;
    static final byte U4 = 3;
    static final byte U5 = 4;
    static final byte U6 = 5;
    static final byte U7 = 6;
    static final byte U8 = 7;
    static final byte U9 = 8;
    static final byte R1 = 9;
    static final byte R2 = 10;
    static final byte R3 = 11;
    static final byte R4 = 12;
    static final byte R5 = 13;
    static final byte R6 = 14;
    static final byte R7 = 15;
    static final byte R8 = 16;
    static final byte R9 = 17;
    static final byte F1 = 18;
    static final byte F2 = 19;
    static final byte F3 = 20;
    static final byte F4 = 21;
    static final byte F5 = 22;
    static final byte F6 = 23;
    static final byte F7 = 24;
    static final byte F8 = 25;
    static final byte F9 = 26;
    static final byte D1 = 27;
    static final byte D2 = 28;
    static final byte D3 = 29;
    static final byte D4 = 30;
    static final byte D5 = 31;
    static final byte D6 = 32;
    static final byte D7 = 33;
    static final byte D8 = 34;
    static final byte D9 = 35;
    static final byte L1 = 36;
    static final byte L2 = 37;
    static final byte L3 = 38;
    static final byte L4 = 39;
    static final byte L5 = 40;
    static final byte L6 = 41;
    static final byte L7 = 42;
    static final byte L8 = 43;
    static final byte L9 = 44;
    static final byte B1 = 45;
    static final byte B2 = 46;
    static final byte B3 = 47;
    static final byte B4 = 48;
    static final byte B5 = 49;
    static final byte B6 = 50;
    static final byte B7 = 51;
    static final byte B8 = 52;
    static final byte B9 = 53;

    //Colors
    static final byte U = 0;
    static final byte R = 1;
    static final byte F = 2;
    static final byte D = 3;
    static final byte L = 4;
    static final byte B = 5;

    static final byte[][] cornerFacelet = {
        { U9, R1, F3 }, { U7, F1, L3 }, { U1, L1, B3 }, { U3, B1, R3 },
        { D3, F9, R7 }, { D1, L9, F7 }, { D7, B9, L7 }, { D9, R9, B7 }
    };
    static final byte[][] edgeFacelet = {
        { U6, R2 }, { U8, F2 }, { U4, L2 }, { U2, B2 }, { D6, R8 }, { D2, F8 },
        { D4, L8 }, { D8, B8 }, { F6, R4 }, { F4, L6 }, { B6, L4 }, { B4, R6 }
    };

    static int[][] Cnk = new int[13][13]; 
    static String[] move2str = {
        "U", "UU", "UUU", "R ", "RR", "R'", "F ", "FF", "FFF",
        "D", "DD", "DDD", "L ", "LL", "LLL", "B", "BB", "BBB"
    };
    static int[] ud2std = {Ux1, Ux2, Ux3, Rx2, Fx2, Dx1, Dx2, Dx3, Lx2, Bx2, Rx1, Rx3, Fx1, Fx3, Lx1, Lx3, Bx1, Bx3};
    static int[] std2ud = new int[18];
    static int[] ckmv2bit = new int[11];

    static class Solution {
        int length = 0;
        int depth1 = 0;
        int verbose = 0;
        int urfIdx = 0;
        int[] moves = new int[31];

        Solution() {}

        void setArgs(int verbose, int urfIdx, int depth1) {
            this.verbose = verbose;
            this.urfIdx = urfIdx;
            this.depth1 = depth1;
        }

        void appendSolMove(int curMove) {
            if (length == 0) {
                moves[length++] = curMove;
                return;
            }
            int axisCur = curMove / 3;
            int axisLast = moves[length - 1] / 3;
            if (axisCur == axisLast) {
                int pow = (curMove % 3 + moves[length - 1] % 3 + 1) % 4;
                if (pow == 3) {
                    length--;
                } else {
                    moves[length - 1] = axisCur * 3 + pow;
                }
                return;
            }
            if (length > 1
                    && axisCur % 3 == axisLast % 3
                    && axisCur == moves[length - 2] / 3) {
                int pow = (curMove % 3 + moves[length - 2] % 3 + 1) % 4;
                if (pow == 3) {
                    moves[length - 2] = moves[length - 1];
                    length--;
                } else {
                    moves[length - 2] = axisCur * 3 + pow;
                }
                return;
            }
            moves[length++] = curMove;
        }

        private void appendQuarterTurn(StringBuilder sb, int move) {
        int axis = move / 3;      
        int pow  = move % 3;     

        char face = "URFDLB".charAt(axis);

        int reps = (pow == 0) ? 1 : (pow == 1 ? 2 : 3);

        for (int i = 0; i < reps; i++) {
            sb.append(face);
        }
    }

        public String toString() {
            StringBuilder sb = new StringBuilder();
            int urf = (verbose & PhaseSolver.INVERSE_SOLUTION) != 0
                    ? (urfIdx + 3) % 6
                    : urfIdx;

            if (urf < 3) {
                // Normal direction
                for (int s = 0; s < length; s++) {
                    int moveStd = Cubie.urfMove[urf][moves[s]];
                    appendQuarterTurn(sb, moveStd);
                }
            } else {
                // Inverse direction
                for (int s = length - 1; s >= 0; s--) {
                    int moveStd = Cubie.urfMove[urf][moves[s]];
                    appendQuarterTurn(sb, moveStd);
                }
            }

            return sb.toString();
            }
        }

    

    static void toCubieCube(byte[] facelets, Cubie out) {
        Arrays.fill(out.ca, (byte) 0);
        Arrays.fill(out.ea, (byte) 0);

        for (byte corner = 0; corner < 8; corner++) {
            byte ori = 0;
            for (; ori < 3; ori++) {
                byte color = facelets[cornerFacelet[corner][ori]];
                if (color == U || color == D) {
                    break;
                }
            }

            byte col1 = facelets[cornerFacelet[corner][(ori + 1) % 3]];
            byte col2 = facelets[cornerFacelet[corner][(ori + 2) % 3]];

            for (byte target = 0; target < 8; target++) {
                if (col1 == cornerFacelet[target][1] / 9 && col2 == cornerFacelet[target][2] / 9) {
                    out.ca[corner] = (byte) ((ori % 3) << 3 | target);
                    break;
                }
            }
        }

        for (byte edge = 0; edge < 12; edge++) {

            byte f0 = facelets[edgeFacelet[edge][0]];
            byte f1 = facelets[edgeFacelet[edge][1]];

            for (byte target = 0; target < 12; target++) {

                byte t0 = (byte) (edgeFacelet[target][0] / 9);
                byte t1 = (byte) (edgeFacelet[target][1] / 9);

                if (f0 == t0 && f1 == t1) {
                    out.ea[edge] = (byte) (target << 1);
                    break;
                }

                if (f0 == t1 && f1 == t0) {
                    out.ea[edge] = (byte) ((target << 1) | 1);
                    break;
                }
            }
        }
    }



    static int getNParity(int idx, int n) {
        int p = 0;
        for (int i = n - 2; i >= 0; i--) {
            p ^= idx % (n - i);
            idx /= (n - i);
        }
        return p & 1;
    }

    static byte setVal(int val0, int val, boolean isEdge) {
        return (byte) (isEdge ? (val << 1 | val0 & 1) : (val | val0 & ~7));
    }

    static int getVal(int val0, boolean isEdge) {
        return isEdge ? val0 >> 1 : val0 & 7;
    }

    static void setNPerm(byte[] arr, int idx, int n, boolean isEdge) {
        long pool = 0xFEDCBA9876543210L;
        long l = 0;
        for (int p = 2; p <= n; p++) {
            l = (l << 4) | (idx % p);
            idx /= p;
        }

        for (int i = 0; i < n - 1; i++) {
            int nibbleShift = ((int) l & 0xF) << 2;
            l >>= 4;

            int val = (int) ((pool >> nibbleShift) & 0xF);
            arr[i] = setVal(arr[i], val, isEdge);

            long mask = (1L << nibbleShift) - 1;
            pool = (pool & mask) | ((pool >> 4) & ~mask);
        }

        arr[n - 1] = setVal(arr[n - 1], (int) (pool & 0xF), isEdge);
    }


    static int getNPerm(byte[] arr, int n, boolean isEdge) {
        int idx = 0;
        long val = 0xFEDCBA9876543210L;
        for (int i = 0; i < n - 1; i++) {
            int digit = getVal(arr[i], isEdge);
            int shift = digit << 2;
            idx = (n - i) * idx + (int) ((val >> shift) & 0xF);
            val -= 0x1111111111111110L << shift;
        }
        return idx;
    }

    static int getComb(byte[] arr, int mask, boolean isEdge) {
        int idx = 0;
        int r = 4;                          
        int end = arr.length - 1;

        for (int i = end; i >= 0; i--) {
            int perm = getVal(arr[i], isEdge);

            if ((perm & 0xC) == mask) {   
                idx += Cnk[i][r--];
            }
        }
        return idx;
    }


    static void setComb(byte[] arr, int idxC, int mask, boolean isEdge) {
        int end = arr.length - 1;
        int r = 4; // items to place
        int nextFill = end;

        for (int i = end; i >= 0; i--) {
            if (idxC >= Cnk[i][r]) {
                idxC -= Cnk[i][r--];
                arr[i] = setVal(arr[i], (r | mask), isEdge);
            } else {
                if ((nextFill & 0xC) == mask) {
                    nextFill -= 4;
                }
                arr[i] = setVal(arr[i], nextFill--, isEdge);
            }
        }
    }


    static {
        for (int i = 0; i < 18; i++) {
            std2ud[ud2std[i]] = i;
        }

        for (int i = 0; i < 10; i++) {

            int ix = ud2std[i] / 3;
            ckmv2bit[i] = 0;

            for (int j = 0; j < 10; j++) {
                int jx = ud2std[j] / 3;
                boolean sameAxis = (ix == jx);
                boolean sameLayer = (ix % 3 == jx % 3) && (ix >= jx);
                ckmv2bit[i] |= (sameAxis || sameLayer ? 1 : 0) << j;
            }
        }

        ckmv2bit[10] = 0;

        for (int i = 0; i < 13; i++) {
            Cnk[i][0] = Cnk[i][i] = 1;
            for (int j = 1; j < i; j++) {
                Cnk[i][j] = Cnk[i - 1][j - 1] + Cnk[i - 1][j];
            }
        }
    }

}
