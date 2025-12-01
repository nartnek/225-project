package rubikscube;

import java.util.Arrays;

class CubeMapping {
    static int[] ud2std = Facelets.UD_TO_STD;
    static int[] std2ud = Facelets.STD_TO_UD;
    static int[] ckmv2bit = Facelets.CK_MV2BIT;
    static int[][] Cnk = Facelets.CNK;
    static String[] move2str = Facelets.MOVE_STRINGS;


    static class Solution {
        int length = 0;
        int depth1 = 0;
        int verbose = 0;
        int urfIdx = 0;
        int[] moves = new int[31];

        Solution() {
        }

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
            int lastMove = moves[length - 1];
            int axisLast = lastMove / 3;

            if (axisCur == axisLast) {
                int powCur = curMove % 3;
                int powLast = lastMove % 3;
                int pow = (powCur + powLast + 1) % 4;

                if (pow == 3) {
                    length--;
                } else {
                    moves[length - 1] = axisCur * 3 + pow;
                }
                return;
            }

            if (length > 1) {
                int prevMove = moves[length - 2];
                int axisPrev = prevMove / 3;

                if (axisCur % 3 == axisLast % 3 && axisCur == axisPrev) {
                    int powCur = curMove % 3;
                    int powPrev = prevMove % 3;
                    int pow = (powCur + powPrev + 1) % 4;

                    if (pow == 3) {
                        moves[length - 2] = moves[length - 1];
                        length--;
                    } else {
                        moves[length - 2] = axisCur * 3 + pow;
                    }
                    return;
                }
            }

            moves[length++] = curMove;
        }


        private void appendQuarterTurn(StringBuilder sb, int move) {
            int axis = move / 3;
            int pow = move % 3;

            char face = "URFDLB".charAt(axis);
            int repetitions = (pow == 0) ? 1 : (pow == 1 ? 2 : 3);

            for (int i = 0; i < repetitions; i++) {
                sb.append(face);
            }
        }

        @Override
        public String toString() {
            StringBuilder sb = new StringBuilder();
            int urf = (verbose & PhaseSolver.INVERSE_SOL) != 0 ? (urfIdx + 3) % 6 : urfIdx;

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
                byte color = facelets[Facelets.CORNER_FACELETS[corner][ori]];
                if (color == Facelets.U || color == Facelets.D) {
                    break;
                }
            }

            byte col1 = facelets[Facelets.CORNER_FACELETS[corner][(ori + 1) % 3]];
            byte col2 = facelets[Facelets.CORNER_FACELETS[corner][(ori + 2) % 3]];

            for (byte target = 0; target < 8; target++) {
                byte base1 = (byte) (Facelets.CORNER_FACELETS[target][1] / 9);
                byte base2 = (byte) (Facelets.CORNER_FACELETS[target][2] / 9);

                if (col1 == base1 && col2 == base2) {
                    int orientation = (ori % 3) << 3;
                    out.ca[corner] = (byte) (orientation | target);
                    break;
                }
            }
        }

        for (byte edge = 0; edge < 12; edge++) {
            byte f0 = facelets[Facelets.EDGE_FACELETS[edge][0]];
            byte f1 = facelets[Facelets.EDGE_FACELETS[edge][1]];

            for (byte target = 0; target < 12; target++) {
                byte t0 = (byte) (Facelets.EDGE_FACELETS[target][0] / 9);
                byte t1 = (byte) (Facelets.EDGE_FACELETS[target][1] / 9);

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
        int parity = 0;

        for (int i = n - 2; i >= 0; i--) {
            parity ^= (idx % (n - i));
            idx /= (n - i);
        }
        return parity & 1;
    }

    static byte setVal(int original, int value, boolean isEdge) {
        if (isEdge) {
            int twist = original & 1;
            return (byte) ((value << 1) | twist);
        } else {
            int oriBits = original & ~7;
            return (byte) (value | oriBits);
        }
    }

    static int getVal(int packed, boolean isEdge) {
        return isEdge ? (packed >>> 1) : (packed & 7);
    }

    static void setNPerm(byte[] arr, int idx, int n, boolean isEdge) {
        long pool = Facelets.PERM_POOL_INIT;
        long digits = 0L;

        for (int p = 2; p <= n; p++) {
            digits = (digits << 4) | (idx % p);
            idx /= p;
        }

        for (int i = 0; i < n - 1; i++) {
            int codeNibble = (int) (digits & 0xF);
            digits >>>= 4;

            int shift = codeNibble << 2;
            int val = (int) ((pool >>> shift) & 0xF);

            arr[i] = setVal(arr[i], val, isEdge);

            long mask = (1L << shift) - 1;
            pool = (pool & mask) | ((pool >>> 4) & ~mask);
        }

        int lastVal = (int) (pool & 0xF);
        arr[n - 1] = setVal(arr[n - 1], lastVal, isEdge);
    }

    static int getNPerm(byte[] arr, int n, boolean isEdge) {
        int idx = 0;
        long pool = Facelets.PERM_POOL_INIT;

        for (int i = 0; i < n - 1; i++) {
            int digit = getVal(arr[i], isEdge);
            int shift = digit << 2;

            idx = (n - i) * idx + (int) ((pool >>> shift) & 0xF);
            pool -= Facelets.PERM_DECR_TEMPLATE << shift;
        }
        return idx;
    }

    static int getComb(byte[] arr, int mask, boolean isEdge) {
        int idx = 0;
        int remainingChosen = 4;
        int end = arr.length - 1;

        for (int i = end; i >= 0; i--) {
            int perm = getVal(arr[i], isEdge);

            if ((perm & 12) == mask) { // 12 == 0xC
                idx += Cnk[i][remainingChosen--];
            }
        }
        return idx;
    }

    static void setComb(byte[] arr, int idxC, int mask, boolean isEdge) {
        int end = arr.length - 1;
        int remainingChosen = 4;
        int nextFill = end;

        for (int i = end; i >= 0; i--) {
            int combVal = Cnk[i][remainingChosen];

            if (idxC >= combVal) {
                idxC -= combVal;
                remainingChosen--;
                arr[i] = setVal(arr[i], (remainingChosen | mask), isEdge);
            } else {
                while ((nextFill & 12) == mask) {
                    nextFill -= 4;
                }
                arr[i] = setVal(arr[i], nextFill--, isEdge);
            }
        }
    }
}
