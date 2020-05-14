import java.io.FileInputStream;
import java.io.IOException;

/**
 * Created by Bratik on 08.06.2017.
 */
public class Samples
{
    public static byte[] buffer;

    public void Samples() { }

    public void initialise(String fileName)
    {
        try(FileInputStream instream=new FileInputStream(fileName) )
        {
            buffer = new byte[instream.available()];
            instream.read(buffer, 0, buffer.length);
        }
        catch(IOException ex)
        {
            System.out.println(ex.getMessage());
        }
    }
}
