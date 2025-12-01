package rubikscube;


import java.io.*;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;

public class Solver {
    public static void main(String[] args) {
        if (args.length < 2) {
            System.out.println("File names are not specified");
            System.out.println("usage: java rubikscube.Solver input_file output_file");
            return;
        }

        String inputPath = args[0];
        String outputPath = args[1];

        try {

            String facelets = readFacelet(inputPath);


            long startTime = System.currentTimeMillis();

            PhaseSolver search = new PhaseSolver();

            String sol = search.solution(facelets, 21, 1_000_000_000L, 0, 0);

            //Debug
            if (sol.startsWith("Error")) {
                System.out.println("Kociemba returned: " + sol);
                writeSolution(outputPath, "");
                return;
            }

            System.out.println("Solution: " + sol);
            long endTime = System.currentTimeMillis();
            System.out.println("Time taken: " + (endTime - startTime) + "ms");

            //Write to output file
            writeSolution(outputPath, sol);

        } catch (IOException e) {
            System.err.println("I/O error: " + e.getMessage());
        }
    }


    private static String readFacelet(String path) throws IOException {
        List<String> lines = Files.readAllLines(Paths.get(path));

        if (lines.size() < 9) {
            throw new IOException("Scramble file has fewer than 9 lines");
        }

        char[] f = new char[54];

        for (int row = 0; row < 3; row++) {
            String line = padRight(lines.get(row), 6);
            for (int col = 0; col < 3; col++) {
                char c = line.charAt(3 + col);
                f[Facelets.U1 + row * 3 + col] = c;
            }
        }

        for (int row = 0; row < 3; row++) {
            String line = padRight(lines.get(3 + row), 12);

            for (int col = 0; col < 3; col++) {
                char c = line.charAt(col);
                f[Facelets.L1 + row * 3 + col] = c;
            }

            for (int col = 0; col < 3; col++) {
                char c = line.charAt(3 + col);
                f[Facelets.F1 + row * 3 + col] = c;
            }

            for (int col = 0; col < 3; col++) {
                char c = line.charAt(6 + col);
                f[Facelets.R1 + row * 3 + col] = c;
            }

            for (int col = 0; col < 3; col++) {
                char c = line.charAt(9 + col);
                f[Facelets.B1 + row * 3 + col] = c;
            }
        }

        for (int row = 0; row < 3; row++) {
            String line = padRight(lines.get(6 + row), 6);
            for (int col = 0; col < 3; col++) {
                char c = line.charAt(3 + col);
                f[Facelets.D1 + row * 3 + col] = c;
            }
        }

        return new String(f);
    }

    private static String padRight(String s, int minLen) {
        if (s.length() >= minLen) return s;
        StringBuilder sb = new StringBuilder(s);
        while (sb.length() < minLen) sb.append(' ');
        return sb.toString();
    }


    private static void writeSolution(String outputPath, String sol) throws IOException {
        try (PrintWriter pw = new PrintWriter(new FileWriter(outputPath))) {
            pw.println(sol);
        }
    }
}
