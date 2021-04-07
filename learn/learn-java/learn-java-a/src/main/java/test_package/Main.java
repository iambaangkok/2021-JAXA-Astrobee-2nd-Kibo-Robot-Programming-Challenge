package test_package;

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
    }
}
