package test_package;

import java.util.Scanner;

public class Main 
{
    public static void main( String[] args )
    {
        byte testByte = 127;
        short testShort = 32767;
        long testLong = 9000000000000L;
        String testString = "GGWP";
        testString += "HH";
        System.out.println( testString.toLowerCase() );
        System.out.println(testByte + testShort + testLong);

        Func01.sayName();
        Func01.sayAge();

        Scanner sc = new Scanner(System.in);
        
        System.out.print("What's your favorite number? : ");
        System.out.println("Oh, It's " + sc.nextDouble() + ". Cool!");

        sc.close();

        testString = "10201";
        System.out.println(Func01.isPalindrome(testString));
        testString = "fuck";
        System.out.println(Func01.isPalindrome(testString));

    }
}
