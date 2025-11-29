
package rubikscube;

public class PhaseSolver {

    public static final boolean USE_TWIST_FLIP_PRUN = true;

  
    static final int MAX_PRE_MOVES = 20;
    static final boolean TRY_INVERSE = true;
    static final boolean TRY_THREE_AXES = true;

    static final boolean USE_COMBP_PRUN = USE_TWIST_FLIP_PRUN;
    static final boolean USE_CONJ_PRUN = USE_TWIST_FLIP_PRUN;
    protected static int MIN_P1LENGTH_PRE = 7;
    protected static int MAX_DEPTH2 = 12;

    static boolean inited = false;

    protected int[] move = new int[31];

    protected Coordinates[] nodeUD = new Coordinates[21];
    protected Coordinates[] nodeRL = new Coordinates[21];
    protected Coordinates[] nodeFB = new Coordinates[21];

    protected long selfSym;
    protected int conjMask;
    protected int urfIdx;
    protected int length1;
    protected int depth1;
    protected int maxDep2;
    protected int solLen;
    protected CubeMapping.Solution solution;
    protected long probe;
    protected long probeMax;
    protected long probeMin;
    protected int verbose;
    protected int valid1;
    protected boolean allowShorter = false;
    protected Cubie cc = new Cubie();
    protected Cubie[] urfCubieCube = new Cubie[6];
    protected Coordinates[] urfCoordCube = new Coordinates[6];
    protected Cubie[] phase1Cubie = new Cubie[21];

    Cubie[] preMoveCubes = new Cubie[MAX_PRE_MOVES + 1];
    int[] preMoves = new int[MAX_PRE_MOVES];
    int preMoveLen = 0;
    int maxPreMoves = 0;

    protected boolean isRec = false;

    public static final int USE_SEPARATOR = 0x1;

    public static final int INVERSE_SOLUTION = 0x2;

    public static final int APPEND_LENGTH = 0x4;

    public static final int OPTIMAL_SOLUTION = 0x8;


    private static final int NUM_NODES = 21;
    private static final int NUM_URF = 6;

    public PhaseSolver() {
        initNodes();
        initUrfStructures();
        initPreMoves();
    }

    private void initNodes() {
        for (int i = 0; i < NUM_NODES; i++) {
            nodeUD[i] = new Coordinates();
            nodeRL[i] = new Coordinates();
            nodeFB[i] = new Coordinates();
            phase1Cubie[i] = new Cubie();
        }
    }

    private void initUrfStructures() {
        for (int i = 0; i < NUM_URF; i++) {
            urfCubieCube[i] = new Cubie();
            urfCoordCube[i] = new Coordinates();
        }
    }

    private void initPreMoves() {
        for (int i = 1; i <= MAX_PRE_MOVES; i++) {
            preMoveCubes[i] = new Cubie();
        }
    }

    public synchronized String solution(
            String facelets,
            int maxDepth,
            long probeMax,
            long probeMin,
            int verbose
    ) {
        int check = verify(facelets);
        if (check != 0) {
            return "Error " + Math.abs(check);
        }

        resetSearchState(maxDepth, probeMax, probeMin, verbose);

        Coordinates.init(false);
        initSearch();

        boolean optimal = (verbose & OPTIMAL_SOLUTION) != 0;
        return optimal ? searchopt() : search();
    }

    private void resetSearchState(int maxDepth, long probeMax, long probeMin, int verbose) {
        this.solLen = maxDepth + 1;
        this.probe = 0;
        this.probeMax = probeMax;
        this.probeMin = Math.min(probeMin, probeMax);
        this.verbose = verbose;

        this.solution = null;
        this.isRec = false;
    }

    protected void initSearch() {

        conjMask = (TRY_INVERSE ? 0 : 0x38) | (TRY_THREE_AXES ? 0 : 0x36);

        selfSym = cc.selfSymmetry();

        if ((selfSym >> 16 & 0xffff) != 0) conjMask |= 0x12;
        if ((selfSym >> 32 & 0xffff) != 0) conjMask |= 0x24;
        if ((selfSym >> 48 & 0xffff) != 0) conjMask |= 0x38;

        selfSym &= 0xffffffffffffL;
        maxPreMoves = conjMask > 7 ? 0 : MAX_PRE_MOVES;

        // Build URF variants
        for (int i = 0; i < NUM_URF; i++) {
            urfCubieCube[i].copy(cc);
            urfCoordCube[i].setWithPrun(urfCubieCube[i], 20);

            cc.URFConjugate();

            if (i % 3 == 2) {
                cc.invCubieCube();
            }
        }
    }

    public synchronized String next(long probeMax, long probeMin, int verbose) {
        this.probe = 0;
        this.probeMax = probeMax;
        this.probeMin = Math.min(probeMin, probeMax);

        boolean previousOptimal = (this.verbose & OPTIMAL_SOLUTION) != 0;
        boolean newOptimal      = (verbose & OPTIMAL_SOLUTION) != 0;

        this.isRec = (previousOptimal == newOptimal);
        this.verbose = verbose;
        this.solution = null;

        return newOptimal ? searchopt() : search();
    }

    public static boolean isInited() {
        return inited;
    }

    public long numProbes() {
        return probe;
    }

    public int length() {
        return solLen;
    }

    public synchronized static void init() {
        Coordinates.init(true);
        inited = true;
    }

    int verify(String facelets) {
        byte[] mapped = new byte[54];
        int colorMask = 0;

        try {
            // Extract center colors in URFDLB order
            String centers = "" +
                    facelets.charAt(CubeMapping.U5) +
                    facelets.charAt(CubeMapping.R5) +
                    facelets.charAt(CubeMapping.F5) +
                    facelets.charAt(CubeMapping.D5) +
                    facelets.charAt(CubeMapping.L5) +
                    facelets.charAt(CubeMapping.B5);

            for (int i = 0; i < 54; i++) {
                int colorIndex = centers.indexOf(facelets.charAt(i));
                if (colorIndex < 0) {
                    return -1; // invalid color
                }
                mapped[i] = (byte) colorIndex;

                // Each color should appear exactly 9 times → 0x999999 mask
                colorMask += 1 << (colorIndex * 4);
            }

        } catch (Exception e) {
            return -1;
        }

        if (colorMask != 0x999999) {
            return -1;
        }

        CubeMapping.toCubieCube(mapped, cc);
        return cc.verify();
    }


    protected int phase1PreMoves(int remaining, int lastMove, Cubie curr, int symMask) {

        preMoveLen = maxPreMoves - remaining;

        boolean shouldSearch =
                isRec ? depth1 == length1 - preMoveLen
                    : (preMoveLen == 0 || (0x36FB7 >> lastMove & 1) == 0);

        if (shouldSearch) {
            depth1 = length1 - preMoveLen;
            phase1Cubie[0] = curr;
            allowShorter = (depth1 == MIN_P1LENGTH_PRE && preMoveLen > 0);

            if (nodeUD[depth1 + 1].setWithPrun(curr, depth1)
                    && phase1(nodeUD[depth1 + 1], symMask, depth1, -1) == 0) {
                return 0;
            }
        }

        // Cannot extend further
        if (remaining == 0 || preMoveLen + MIN_P1LENGTH_PRE >= length1) {
            return 1;
        }

        // Determine moves to skip
        int skipMoves = Cubie.getSkipMoves(symMask);

        if (remaining == 1 || preMoveLen + 1 + MIN_P1LENGTH_PRE >= length1) {
            skipMoves |= 0x36FB7;
        }

        lastMove = (lastMove / 3) * 3;  // normalize move group base

        for (int m = 0; m < 18; m++) {

            // Skip same-axis move group
            if (m == lastMove || m == lastMove - 9 || m == lastMove + 9) {
                m += 2;
                continue;
            }

            // Skip illegal or disallowed recursive moves
            if ((isRec && m != preMoves[maxPreMoves - remaining])
                    || (skipMoves & (1 << m)) != 0) {
                continue;
            }

            // Apply move
            Cubie next = preMoveCubes[remaining];
            Cubie.CornMult(Cubie.moveCube[m], curr, next);
            Cubie.EdgeMult(Cubie.moveCube[m], curr, next);

            preMoves[maxPreMoves - remaining] = m;

            int ret = phase1PreMoves(remaining - 1, m, next,
                    symMask & (int) Cubie.moveCubeSym[m]);

            if (ret == 0) {
                return 0;
            }
        }

        return 1;
    }


    protected String search() {
        for (length1 = isRec ? length1 : 0; length1 < solLen; length1++) {
            maxDep2 = Math.min(MAX_DEPTH2, solLen - length1 - 1);

            for (urfIdx = isRec ? urfIdx : 0; urfIdx < 6; urfIdx++) {

                if ((conjMask & (1 << urfIdx)) != 0) {
                    continue;
                }

                int ok = phase1PreMoves(maxPreMoves, -30, urfCubieCube[urfIdx],
                                        (int) (selfSym & 0xffff));

                if (ok == 0) {
                    return solution == null ? "Error 8" : solution.toString();
                }
            }
        }
        return solution == null ? "Error 7" : solution.toString();
    }



    protected int initPhase2Pre() {
        isRec = false;
        if (probe >= (solution == null ? probeMax : probeMin)) {
            return 0;
        }
        ++probe;

        for (int i = valid1; i < depth1; i++) {
            Cubie.CornMult(phase1Cubie[i], Cubie.moveCube[move[i]], phase1Cubie[i + 1]);
            Cubie.EdgeMult(phase1Cubie[i], Cubie.moveCube[move[i]], phase1Cubie[i + 1]);
        }
        valid1 = depth1;

        int p2corn = phase1Cubie[depth1].getCPermSym();
        int p2csym = p2corn & 0xf;
        p2corn >>= 4;
        int p2edge = phase1Cubie[depth1].getEPermSym();
        int p2esym = p2edge & 0xf;
        p2edge >>= 4;
        int p2mid = phase1Cubie[depth1].getMPerm();
        int edgei = Cubie.getPermSymInv(p2edge, p2esym, false);
        int corni = Cubie.getPermSymInv(p2corn, p2csym, true);

        int lastMove = depth1 == 0 ? -1 : move[depth1 - 1];
        int lastPre = preMoveLen == 0 ? -1 : preMoves[preMoveLen - 1];

        int ret = 0;
        int p2switchMax = (preMoveLen == 0 ? 1 : 2) * (depth1 == 0 ? 1 : 2);
        for (int p2switch = 0, p2switchMask = (1 << p2switchMax) - 1;
                p2switch < p2switchMax; p2switch++) {
            // 0 normal; 1 lastmove; 2 lastmove + premove; 3 premove
            if ((p2switchMask >> p2switch & 1) != 0) {
                p2switchMask &= ~(1 << p2switch);
                ret = initPhase2(p2corn, p2csym, p2edge, p2esym, p2mid, edgei, corni);
                if (ret == 0 || ret > 2) {
                    break;
                } else if (ret == 2) {
                    p2switchMask &= 0x4 << p2switch; // 0->2; 1=>3; 2=>N/A
                }
            }
            if (p2switchMask == 0) {
                break;
            }
            if ((p2switch & 1) == 0 && depth1 > 0) {
                int m = CubeMapping.std2ud[lastMove / 3 * 3 + 1];
                move[depth1 - 1] = CubeMapping.ud2std[m] * 2 - move[depth1 - 1];

                p2mid = Coordinates.MPermMove[p2mid][m];
                p2corn = Coordinates.CPermMove[p2corn][Cubie.SymMoveUD[p2csym][m]];
                p2csym = Cubie.SymMult[p2corn & 0xf][p2csym];
                p2corn >>= 4;
                p2edge = Coordinates.EPermMove[p2edge][Cubie.SymMoveUD[p2esym][m]];
                p2esym = Cubie.SymMult[p2edge & 0xf][p2esym];
                p2edge >>= 4;
                corni = Cubie.getPermSymInv(p2corn, p2csym, true);
                edgei = Cubie.getPermSymInv(p2edge, p2esym, false);
            } else if (preMoveLen > 0) {
                int m = CubeMapping.std2ud[lastPre / 3 * 3 + 1];
                preMoves[preMoveLen - 1] = CubeMapping.ud2std[m] * 2 - preMoves[preMoveLen - 1];

                p2mid = Cubie.MPermInv[Coordinates.MPermMove[Cubie.MPermInv[p2mid]][m]];
                p2corn = Coordinates.CPermMove[corni >> 4][Cubie.SymMoveUD[corni & 0xf][m]];
                corni = p2corn & ~0xf | Cubie.SymMult[p2corn & 0xf][corni & 0xf];
                p2corn = Cubie.getPermSymInv(corni >> 4, corni & 0xf, true);
                p2csym = p2corn & 0xf;
                p2corn >>= 4;
                p2edge = Coordinates.EPermMove[edgei >> 4][Cubie.SymMoveUD[edgei & 0xf][m]];
                edgei = p2edge & ~0xf | Cubie.SymMult[p2edge & 0xf][edgei & 0xf];
                p2edge = Cubie.getPermSymInv(edgei >> 4, edgei & 0xf, false);
                p2esym = p2edge & 0xf;
                p2edge >>= 4;
            }
        }
        if (depth1 > 0) {
            move[depth1 - 1] = lastMove;
        }
        if (preMoveLen > 0) {
            preMoves[preMoveLen - 1] = lastPre;
        }
        return ret == 0 ? 0 : 2;
    }

    protected int initPhase2(int p2corn, int p2csym, int p2edge, int p2esym, int p2mid, int edgei, int corni) {
        int prun = Math.max(
                       Coordinates.getPruning(Coordinates.EPermCCombPPrun,
                                            (edgei >> 4) * Coordinates.N_COMB + Coordinates.CCombPConj[Cubie.Perm2CombP[corni >> 4] & 0xff][Cubie.SymMultInv[edgei & 0xf][corni & 0xf]]),
                       Math.max(
                           Coordinates.getPruning(Coordinates.EPermCCombPPrun,
                                                p2edge * Coordinates.N_COMB + Coordinates.CCombPConj[Cubie.Perm2CombP[p2corn] & 0xff][Cubie.SymMultInv[p2esym][p2csym]]),
                           Coordinates.getPruning(Coordinates.MCPermPrun,
                                                p2corn * Coordinates.N_MPERM + Coordinates.MPermConj[p2mid][p2csym])));

        if (prun > maxDep2) {
            return prun - maxDep2;
        }

        int depth2;
        for (depth2 = maxDep2; depth2 >= prun; depth2--) {
            int ret = phase2(p2edge, p2esym, p2corn, p2csym, p2mid, depth2, depth1, 10);
            if (ret < 0) {
                break;
            }
            depth2 -= ret;
            solLen = 0;
            solution = new CubeMapping.Solution();
            solution.setArgs(verbose, urfIdx, depth1);
            for (int i = 0; i < depth1 + depth2; i++) {
                solution.appendSolMove(move[i]);
            }
            for (int i = preMoveLen - 1; i >= 0; i--) {
                solution.appendSolMove(preMoves[i]);
            }
            solLen = solution.length;
        }

        if (depth2 != maxDep2) { //At least one solution has been found.
            maxDep2 = Math.min(MAX_DEPTH2, solLen - length1 - 1);
            return probe >= probeMin ? 0 : 1;
        }
        return 1;
    }

    protected int phase1(Coordinates node, int ssym, int maxl, int lastAxis) {

        // Early exit: pruning reached and near the end of phase1
        if (node.prun == 0 && maxl < 5) {
            if (allowShorter || maxl == 0) {
                depth1 -= maxl;
                int ret = initPhase2Pre();
                depth1 += maxl;
                return ret;
            }
            return 1;
        }

        int skipMask = Cubie.getSkipMoves(ssym);

        for (int axis = 0; axis < 18; axis += 3) {

            // Skip same-axis repetition
            if (axis == lastAxis || axis == lastAxis - 9) {
                continue;
            }

            for (int power = 0; power < 3; power++) {
                int moveIdx = axis + power;

                // Skip invalid recursive branches or masked moves
                if ((isRec && moveIdx != move[depth1 - maxl])
                        || ((skipMask & (1 << moveIdx)) != 0)) {
                    continue;
                }

                // Pruning check
                int pr = nodeUD[maxl].pruningMoves(node, moveIdx, true);
                if (pr > maxl) break;
                if (pr == maxl) continue;

                // Optional symmetry pruning
                if (USE_CONJ_PRUN) {
                    pr = nodeUD[maxl].pruningMovesConj(node, moveIdx);
                    if (pr > maxl) break;
                    if (pr == maxl) continue;
                }

                // Apply move and recurse
                move[depth1 - maxl] = moveIdx;
                valid1 = Math.min(valid1, depth1 - maxl);

                int ret = phase1(nodeUD[maxl],
                                ssym & (int) Cubie.moveCubeSym[moveIdx],
                                maxl - 1,
                                axis);

                if (ret == 0) return 0;
                if (ret >= 2) break;
            }
        }
        return 1;
    }


    protected String searchopt() {

        int maxPrunURF  = 0;
        int maxPrunInv  = 0;

        // Compute pruning for urf 0–5
        for (int i = 0; i < 6; i++) {
            urfCoordCube[i].calcPruning(false);
            if (i < 3) {
                maxPrunURF = Math.max(maxPrunURF, urfCoordCube[i].prun);
            } else {
                maxPrunInv = Math.max(maxPrunInv, urfCoordCube[i].prun);
            }
        }

        // Choose best orientation (normal or inverted)
        urfIdx = maxPrunInv > maxPrunURF ? 3 : 0;
        phase1Cubie[0] = urfCubieCube[urfIdx];

        for (length1 = isRec ? length1 : 0; length1 < solLen; length1++) {

            Coordinates ud = urfCoordCube[urfIdx + 0];
            Coordinates rl = urfCoordCube[urfIdx + 1];
            Coordinates fb = urfCoordCube[urfIdx + 2];

            boolean prunable =
                    ud.prun <= length1 &&
                    rl.prun <= length1 &&
                    fb.prun <= length1;

            if (prunable && phase1opt(ud, rl, fb, selfSym, length1, -1) == 0) {
                return solution == null ? "Error " : solution.toString();
            }
        }

        return solution == null ? "Error, no sol" : solution.toString();
    }


    protected int phase1opt(Coordinates ud, Coordinates rl, Coordinates fb, long ssym, int maxl, int lastAxis) {

        if (ud.prun == 0 && rl.prun == 0 && fb.prun == 0 && maxl < 5) {
            maxDep2 = maxl;
            depth1 = length1 - maxl;
            return (initPhase2Pre() == 0) ? 0 : 1;
        }

        int skipMask = Cubie.getSkipMoves(ssym);

        for (int axis = 0; axis < 18; axis += 3) {

            if (axis == lastAxis || axis == lastAxis - 9) {
                continue;
            }

            for (int p = 0; p < 3; p++) {

                int m = axis + p;

                if ((isRec && m != move[length1 - maxl])
                        || (skipMask & (1 << m)) != 0) {
                    continue;
                }

                int prUD = nodeUD[maxl].pruningMoves(ud, m, false);
                if (USE_CONJ_PRUN) {
                    prUD = Math.max(prUD, nodeUD[maxl].pruningMovesConj(ud, m));
                }
                if (prUD > maxl) break;
                if (prUD == maxl) continue;

                m = Cubie.urfMove[2][m];
                int prRL = nodeRL[maxl].pruningMoves(rl, m, false);
                if (USE_CONJ_PRUN) {
                    prRL = Math.max(prRL, nodeRL[maxl].pruningMovesConj(rl, m));
                }
                if (prRL > maxl) break;
                if (prRL == maxl) continue;

                m = Cubie.urfMove[2][m];
                int prFB = nodeFB[maxl].pruningMoves(fb, m, false);
                if (USE_CONJ_PRUN) {
                    prFB = Math.max(prFB, nodeFB[maxl].pruningMovesConj(fb, m));
                }

                if (prUD == prRL && prRL == prFB && prFB != 0) {
                    prFB++;
                }

                if (prFB > maxl) break;
                if (prFB == maxl) continue;

                m = Cubie.urfMove[2][m];

                move[length1 - maxl] = m;
                valid1 = Math.min(valid1, length1 - maxl);

                int ret = phase1opt(nodeUD[maxl], nodeRL[maxl], nodeFB[maxl],ssym & Cubie.moveCubeSym[m],maxl - 1, axis);

                if (ret == 0) return 0;
            }
        }

        return 1;
    }


    protected int phase2(int edge, int esym, int corn, int csym, int mid, int maxl, int depth, int lastMove){
        // Solved state
        if (edge == 0 && corn == 0 && mid == 0) {
            return maxl;
        }

        int moveMask = CubeMapping.ckmv2bit[lastMove];

        for (int m = 0; m < 10; m++) {

            if ((moveMask >> m & 1) != 0) {
                m += (0x42 >> m) & 3;
                continue;
            }

            // Apply UD move m to all coordinates
            int mid2  = Coordinates.MPermMove[mid][m];
            int corn2 = Coordinates.CPermMove[corn][Cubie.SymMoveUD[csym][m]];
            int csym2 = Cubie.SymMult[corn2 & 0xf][csym]; corn2 >>= 4;
            int edge2 = Coordinates.EPermMove[edge][Cubie.SymMoveUD[esym][m]];
            int esym2 = Cubie.SymMult[edge2 & 0xf][esym]; edge2 >>= 4;

            int edgeInv = Cubie.getPermSymInv(edge2, esym2, false);
            int cornInv = Cubie.getPermSymInv(corn2, csym2, true);

            // First pruning check
            int pr = Coordinates.getPruning(
                    Coordinates.EPermCCombPPrun,
                    (edgeInv >> 4) * Coordinates.N_COMB
                            + Coordinates.CCombPConj[
                                    Cubie.Perm2CombP[cornInv >> 4] & 0xff
                            ][Cubie.SymMultInv[edgeInv & 0xf][cornInv & 0xf]]
            );

            if (pr > maxl + 1) return maxl - pr + 1;
            if (pr >= maxl) {
                m += (0x42 >> m & 3) & (maxl - pr);
                continue;
            }

            // Secondary pruning
            pr = Math.max(
                    Coordinates.getPruning(Coordinates.MCPermPrun,
                            corn2 * Coordinates.N_MPERM + Coordinates.MPermConj[mid2][csym2]),
                    Coordinates.getPruning(Coordinates.EPermCCombPPrun,
                            edge2 * Coordinates.N_COMB + Coordinates.CCombPConj[
                                    Cubie.Perm2CombP[corn2] & 0xff
                            ][Cubie.SymMultInv[esym2][csym2]])
            );

            if (pr >= maxl) {
                m += (0x42 >> m & 3) & (maxl - pr);
                continue;
            }

            // Recurse deeper
            int ret = phase2(edge2, esym2, corn2, csym2, mid2,
                            maxl - 1, depth + 1, m);

            if (ret >= 0) {
                move[depth] = CubeMapping.ud2std[m];
                return ret;
            }

            if (ret < -2) break;
            if (ret < -1) {
                m += (0x42 >> m) & 3;
            }
        }

        return -1;
    }

}
