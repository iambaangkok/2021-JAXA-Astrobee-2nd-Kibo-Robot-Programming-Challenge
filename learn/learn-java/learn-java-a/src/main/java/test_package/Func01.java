package test_package;

public class Func01 {
    public static int mod17(int num) {
        return num%17;
    }

    public static void sayName(){
        System.out.println("BK");
    }
    public static void sayAge(){
        System.out.println(19);
    }

    public static boolean isPalindrome(String str){
        boolean check = true;
        for(int i = 0 ; i < str.length()/2; ++i){
            if(str.charAt(i) != str.charAt(str.length()-1-i)){
                check = false;
                break;
            }
        }

        return check;
    }
}
