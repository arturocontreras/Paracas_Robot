using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Security.Permissions;
using System.Runtime.InteropServices;
using System.Data.SqlClient;
using System.IO.Ports;
using System.Threading;


namespace WindowsFormsApplication9
{
    [ComVisibleAttribute(true)]
    [PermissionSet(SecurityAction.Demand, Name = "FullTrust")]

    public partial class RP : Form
    {
        //Comunicacion serial
        int numero_prueba = 0;
        static SerialPort port;
        string dato_serial;
        int contador_serial_event = 0;


        //Coordenadas
        double[] latitudes = new double[100];
        double[] longitudes = new double[100];
        int[] a1 = new int[100];
        int[] b1 = new int[100];
        int[] c1 = new int[100];
        int[] d1 = new int[100];
        int[] a2 = new int[100];
        int[] b2 = new int[100];
        int[] c2 = new int[100];
        int[] d2 = new int[100];
        string recibidos;
        int[] buffer = new int[1000];
        byte[] buffer_byte = new byte[1];
        int contador = 0;
        int contador_ruteo = 0;

        int num_columnas = 8;
        int wp_permitido_flag = 0;
        double[,] Xcorners = new double[15, 7];
        double[,] Ycorners = new double[15, 7];

        string numeros_x, numeros_y;
        double[] X1 = new double[16];
        double[] X2 = new double[16];
        double[] X3 = new double[16];
        double[] X4 = new double[16];
        double[] X5 = new double[16];
        double[] X6 = new double[16];
        double[] X7 = new double[16];
        double[] X8 = new double[16];
        double[] X9 = new double[16];
        double[] X10 = new double[16];
        double[] X11 = new double[16];
        double[] X12 = new double[16];
        double[] X13 = new double[16];
        double[] X14 = new double[16];
        double[] X15 = new double[16];
        double[] X16 = new double[16];
        double[] X17 = new double[16];
        double[] X18 = new double[16];

        double[] Y1 = new double[16];
        double[] Y2 = new double[16];
        double[] Y3 = new double[16];
        double[] Y4 = new double[16];
        double[] Y5 = new double[16];
        double[] Y6 = new double[16];
        double[] Y7 = new double[16];
        double[] Y8 = new double[16];
        double[] Y9 = new double[16];
        double[] Y10 = new double[16];
        double[] Y11 = new double[16];
        double[] Y12 = new double[16];
        double[] Y13 = new double[16];
        double[] Y14 = new double[16];
        double[] Y15 = new double[16];
        double[] Y16 = new double[16];
        double[] Y17 = new double[16];
        double[] Y18 = new double[16];

        double[] cc1 = new double[16];
        double[] cc2 = new double[16];
        double[] cc3 = new double[16];
        double[] cc4 = new double[16];
        double[] cc5 = new double[16];
        double[] cc6 = new double[16];
        double[] cc7 = new double[16];
        double[] cc8 = new double[16];
        double[] cc9 = new double[16];
        double[] cc10 = new double[16];
        double[] cc11 = new double[16];
        double[] cc12 = new double[16];
        double[] cc13 = new double[16];
        double[] cc14 = new double[16];
        double[] cc15 = new double[16];
        double[] cc16 = new double[16];
        double[] cc17 = new double[16];
        double[] cc18 = new double[16];

        double[] pendientes = new double[16];
        double pendiente;
        double offset;
        //double pendiente1, pendiente2, pendiente3, pendiente4, pendiente5, pendiente6, pendiente7, pendiente8, pendiente9, pendiente10, pendiente11, pendiente12, pendiente13, pendiente14, pendiente15, pendiente16, pendiente17, pendiente18;
        //double offset1, offset2, offset3, offset4, offset5, offset6, offset7, offset8, offset9, offset10, offset11, offset12, offset13, offset14, offset15, offset16, offset17, offset18;


        double[] Lats_extremas = new double[60];
        double[] Lons_extremas = new double[60];

        //******************  AUTOPATH ****************************
        double[] Xautop = new double[100];
        double[] Yautop = new double[100];
        double lat;
        double lon;
        double modulo;
        double radio;//error entre posicion y Corner mas cercano
        double radio2;//error entre posicion y Corner mas cercano
        int col_elegida;
        int fil_elegida;
        int col_elegida_inicio;
        int fil_elegida_inicio;
        int size_path;
        //********************COMUNICACIONES**************************************
        byte[] byte_buffer = new byte[100];
        int bytes;

        //*******************RECEPCION DATOS**************************************
        int lata, latb, latc, latd, lona, lonb, lonc, lond, yaw_raw, obs;
        double latitud_actual, longitud_actual,yaw,i_tot,v_tot,nivel_tank,hum_amb,temp_amb,rad_sol,hum_sue,pH_sue;

        public RP()
        {
            InitializeComponent();

            var port = new SerialPort();
            puertos_baudios.SelectedItem = "115200";

            panel1.Visible = true;
            panel2.Visible = false;
            panel3.Visible = false;
            panel4.Visible = false;
            panel5.Visible = false;
            panel6.Visible = false;
            panel7.Visible = false;
            panel8.Visible = false;

            panel1.Dock = DockStyle.Fill;

            mapeo();
            lugar_geometrico();
            dibujo_limites(); 

        }


        public void coordenadas(double latitude, double longitude)
        {
            double lat_round, lon_round;

            lon_round = Math.Round(-longitude * 100, 4);
            lat_round = Math.Round(-latitude * 100, 4);

            //monitor.Text += "P" + contador + ": Latitud " + latitude.ToString() + "  Longitud " + longitude.ToString() + "\n";
            wp_permitido_flag = 0;

            double c_prueba;
            for(int k=0; k<num_columnas;k++){

                c_prueba = latitude - pendientes[k] * longitude;

                switch (k)
                {
                    case 0:  
                    if ((c_prueba >= menor(cc1)) && (c_prueba <= mayor(cc1)))
                    {
                        wp_permitido_flag = 0;

                    }
                    else wp_permitido_flag += 1;


                    break;

                    case 1:
                    if ((c_prueba >= menor(cc2)) && (c_prueba <= mayor(cc2)))
                    {
                        wp_permitido_flag = 0;
                    }
                    else wp_permitido_flag += 1;

                    break;

                    case 2:
                    if ((c_prueba >= menor(cc3)) && (c_prueba <= mayor(cc3)))
                    {
                        wp_permitido_flag = 0;
                    }
                    else wp_permitido_flag += 1;

                    break;

                    case 3:
                    if ((c_prueba >= menor(cc4)) && (c_prueba <= mayor(cc4)))
                    {
                        wp_permitido_flag = 0;
                    }
                    else wp_permitido_flag += 1;

                    break;

                    case 4:
                    if ((c_prueba >= menor(cc5)) && (c_prueba <= mayor(cc5)))
                    {
                        wp_permitido_flag = 0;
                    }
                    else wp_permitido_flag += 1;

                    break;

                    case 5:
                    if ((c_prueba >= menor(cc6)) && (c_prueba <= mayor(cc6)))
                    {
                        wp_permitido_flag = 0;
                    }
                    else wp_permitido_flag += 1;

                    break;

                    case 6:
                    if ((c_prueba >= menor(cc7)) && (c_prueba <= mayor(cc7)))
                    {
                        wp_permitido_flag = 0;
                    }
                    else wp_permitido_flag += 1;

                    break;

                    case 7:
                    if ((c_prueba >= menor(cc8)) && (c_prueba <= mayor(cc8)))
                    {
                        wp_permitido_flag = 0;
                    }
                    else wp_permitido_flag += 1;

                    break;

                }
                
            }

            if (wp_permitido_flag < num_columnas)
            {
                wp_permitido_flag = 0;
                testeador.Text = "Punto no permitido";
            }
            else
            {
                contador++;
                monitor.Text += longitude.ToString() + " , " + latitude.ToString() + "\n";
                testeador.Text = "Punto permitido";

                latitudes[contador] = lat_round;
                longitudes[contador] = lon_round;

                wp_permitido_flag = 1;
            }

            testeador2.Text = wp_permitido_flag.ToString();
            webBrowser2.Document.InvokeScript("wp_permitido", new string[] { wp_permitido_flag.ToString() });

        }

        public void ruteo(double latitude, double longitude)
        {
            lat = latitude;
            lon = longitude;
            //modulo = Math.Pow(latitude,2) + Math.Pow(longitude,2) ;

            radio = Math.Pow((longitude - Xcorners[0, 0]), 2) + Math.Pow((latitude - Ycorners[0, 0]), 2);

            col_elegida = 0;
            fil_elegida = 0;


            for (int col = 0; col < num_columnas -1 ; col++) //Cantidad de columnas de matriz Cx-Cy
            {
                for (int fil = 0; fil < 15; fil++) //Cantidad de filas de matriz Cx-Cy 
                {
                    radio2 = Math.Pow((lat - Ycorners[fil, col]), 2) + Math.Pow((lon - Xcorners[fil, col]), 2);

                    if (radio > radio2)
                    {
                        col_elegida = col;
                        fil_elegida = fil;

                        radio = radio2;
                    }

                }
            }

            //rutear.Text = "El corner elegido es : " + fil_elegida.ToString() + "   " + col_elegida.ToString() ;

            contador_ruteo++;
            if (contador_ruteo == 2)
            {
                rutear.Text = "Punto INICIO ingresado:" + "El corner elegido es : " + fil_elegida.ToString() + "   " + col_elegida.ToString();

                Xautop[0] = lon;
                Yautop[0] = lat;

                col_elegida_inicio = col_elegida;
                fil_elegida_inicio = fil_elegida;

            }

            if (contador_ruteo == 3)
            {
                rutear.Text = "Punto FINAL ingresado:" + "El corner elegido es : " + fil_elegida.ToString() + "   " + col_elegida.ToString();

                if (col_elegida > col_elegida_inicio)
                {
                    for (int cols = col_elegida_inicio; cols <= col_elegida; cols++)
                    {
                        size_path++;
                        Xautop[size_path] = Xcorners[fil_elegida_inicio, cols];
                        Yautop[size_path] = Ycorners[fil_elegida_inicio, cols];
                    }

                }
                else 
                {
                    for (int cols = col_elegida_inicio; cols >= col_elegida; cols--)
                    {
                        size_path++;
                        Xautop[size_path] = Xcorners[fil_elegida_inicio, cols];
                        Yautop[size_path] = Ycorners[fil_elegida_inicio, cols];
                    }
                }


                if (fil_elegida > fil_elegida_inicio)
                {
                    for (int fils = fil_elegida_inicio; fils <= fil_elegida; fils++)
                    {
                        size_path++;
                        Xautop[size_path] = Xcorners[fils, col_elegida];
                        Yautop[size_path] = Ycorners[fils, col_elegida];
                    }

                }
                else
                {
                    for (int fils = fil_elegida_inicio; fils >= fil_elegida; fils--)
                    {
                        size_path++;
                        Xautop[size_path] = Xcorners[fils, col_elegida];
                        Yautop[size_path] = Ycorners[fils, col_elegida];
                    }
                }

                size_path++;

                Xautop[size_path] = lon;
                Yautop[size_path] = lat;

                for (int j = 0; j < size_path; j++)
                {
                    webBrowser2.Document.InvokeScript("Px_to_corner", new string[] { Yautop[j].ToString(), Yautop[j + 1].ToString(), Xautop[j].ToString(), Xautop[j + 1].ToString() });
                }


                //Reseteando variables
                size_path = 0;
                contador_ruteo = 1;

            }

            testeador_x.Text = contador_ruteo.ToString();
         
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            this.WindowState = FormWindowState.Maximized;
            webBrowser1.ObjectForScripting = this;
            webBrowser2.ObjectForScripting = this;
            //alarmas();
            //puertos serial
            var ports = SerialPort.GetPortNames();
            puertos_serial.DataSource = ports;
        }

        void alarmas()
        {

            int n = dataGridView1.Rows.Add();

            dataGridView1.Rows[n].Cells[0].Value = n +1;
            dataGridView1.Rows[n].Cells[1].Value = "Sobretemperatura driver";
            dataGridView1.Rows[n].Cells[2].Value = System.DateTime.Now;
            dataGridView1.Rows[n].Cells[3].Value = 2;
            dataGridView1.Rows[n].Cells[4].Value = "Reducir la carga sobre el robot";

            dataGridView1.Rows[n].Cells[2].ValueType = typeof(DateTime);
            Font alarma_letras = new Font("Arial", 12, FontStyle.Bold);
            dataGridView1.Rows[n].Cells[0].Style.Font = alarma_letras;
            dataGridView1.Rows[n].Cells[0].Style.ForeColor = Color.Red;
            dataGridView1.Rows[n].Cells[0].Style.BackColor = Color.Black;
            dataGridView1.Rows[n].Cells[1].Style.Font = alarma_letras;
            dataGridView1.Rows[n].Cells[1].Style.ForeColor = Color.Red;
            dataGridView1.Rows[n].Cells[1].Style.BackColor = Color.Black;
            dataGridView1.Rows[n].Cells[2].Style.Font = alarma_letras;
            dataGridView1.Rows[n].Cells[2].Style.ForeColor = Color.Red;
            dataGridView1.Rows[n].Cells[2].Style.BackColor = Color.Black;
            dataGridView1.Rows[n].Cells[3].Style.Font = alarma_letras;
            dataGridView1.Rows[n].Cells[3].Style.ForeColor = Color.Red;
            dataGridView1.Rows[n].Cells[3].Style.BackColor = Color.Black;
            dataGridView1.Rows[n].Cells[4].Style.Font = alarma_letras;
            dataGridView1.Rows[n].Cells[4].Style.ForeColor = Color.Red;
            dataGridView1.Rows[n].Cells[4].Style.BackColor = Color.Black;

        }

        private void monitoreoToolStripMenuItem_Click(object sender, EventArgs e)
        {
            panel1.Visible = true;
            panel2.Visible = false;
            panel3.Visible = false;
            panel4.Visible = false;
            panel5.Visible = false;
            panel6.Visible = false;
            panel7.Visible = false;
            panel8.Visible = false;

            panel1.Dock = DockStyle.Fill;

        }

        private void button1_Click(object sender, EventArgs e)
        {

        }

        private void controlRemotoToolStripMenuItem_Click(object sender, EventArgs e)
        {
            panel1.Visible = false;
            panel2.Visible = false;
            panel3.Visible = true;
            panel4.Visible = false;
            panel5.Visible = false;
            panel6.Visible = false;
            panel7.Visible = false;
            panel8.Visible = false;

            panel3.Dock = DockStyle.Fill;
   
        }

        private void panel2_Paint(object sender, PaintEventArgs e)
        {

        }

        private void testeoToolStripMenuItem_Click(object sender, EventArgs e)
        {
            panel1.Visible = false;
            panel2.Visible = false;
            panel3.Visible = false;
            panel4.Visible = true;
            panel5.Visible = false;
            panel6.Visible = false;
            panel7.Visible = false;
            panel8.Visible = false;

            panel4.Dock = DockStyle.Fill;
        }

        private void button1_Click_1(object sender, EventArgs e)
        {

        }

        private void mapaToolStripMenuItem_Click(object sender, EventArgs e)
        {
            panel1.Visible = false;
            panel2.Visible = true;
            panel3.Visible = false;
            panel4.Visible = false;
            panel5.Visible = false;
            panel6.Visible = false;
            panel7.Visible = false;
            panel8.Visible = false;

            panel2.Dock = DockStyle.Fill;
        }

        private void button2_Click_1(object sender, EventArgs e)
        {

        }

        private void panel2_Paint_1(object sender, PaintEventArgs e)
        {

        }

        private void button7_Click(object sender, EventArgs e)
        {

        }

        private void calibracionesToolStripMenuItem_Click(object sender, EventArgs e)
        {
            panel1.Visible = false;
            panel2.Visible = false;
            panel3.Visible = false;
            panel4.Visible = false;
            panel5.Visible = true;
            panel6.Visible = false;
            panel7.Visible = false;
            panel8.Visible = false;

            panel5.Dock = DockStyle.Fill;
        }

        private void alarmasToolStripMenuItem_Click(object sender, EventArgs e)
        {
            panel1.Visible = false;
            panel2.Visible = false;
            panel3.Visible = false;
            panel4.Visible = false;
            panel5.Visible = false;
            panel6.Visible = true;
            panel7.Visible = false;
            panel8.Visible = false;

            panel6.Dock = DockStyle.Fill;
        }

        private void gráficasToolStripMenuItem_Click(object sender, EventArgs e)
        {
            panel1.Visible = false;
            panel2.Visible = false;
            panel3.Visible = false;
            panel4.Visible = false;
            panel5.Visible = false;
            panel6.Visible = false;
            panel7.Visible = true;
            panel8.Visible = false;

            panel7.Dock = DockStyle.Fill;
        }

        private void acercToolStripMenuItem_Click(object sender, EventArgs e)
        {
            panel1.Visible = false;
            panel2.Visible = false;
            panel3.Visible = false;
            panel4.Visible = false;
            panel5.Visible = false;
            panel6.Visible = false;
            panel7.Visible = false;
            panel8.Visible = true;

            panel8.Dock = DockStyle.Fill;
        }

        private void label13_Click(object sender, EventArgs e)
        {

        }

        private void label18_Click(object sender, EventArgs e)
        {

        }

        private void checkedListBox1_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        private void tableLayoutPanel2_Paint(object sender, PaintEventArgs e)
        {

        }

        private void panel2_Paint_2(object sender, PaintEventArgs e)
        {

        }

        private void tableLayoutPanel2_Paint_1(object sender, PaintEventArgs e)
        {

        }

        private void button2_Click(object sender, EventArgs e)
        {
            webBrowser1.Document.InvokeScript("mostrarcoordenada", new string[] { lat_in.Text, lon_in.Text });
        }

        private void label37_Click(object sender, EventArgs e)
        {

        }

        private void label41_Click(object sender, EventArgs e)
        {

        }

        private void panel6_Paint(object sender, PaintEventArgs e)
        {

        }

        private void dataGridView1_CellContentClick(object sender, DataGridViewCellEventArgs e)
        {

        }

        private void button4_Click(object sender, EventArgs e)
        {
            alarmas();
        }


        private void trackBar1_MouseDown(object sender, MouseEventArgs e)
        {
            label_vel.Text = trackBar1.Value.ToString();

            //MessageBox.Show(trackBar1.Value.ToString()); 
        }

        private void richTextBox7_KeyPress(object sender, KeyPressEventArgs e)
        {
            byte[] comando = new byte[1];

            switch (e.KeyChar) {

                case 'w' :
                comando[0] = Convert.ToByte(119);//cabecera 'w'
                port.Write(comando, 0, comando.Length);

                numero_prueba++;
                label_prueba.Text = (numero_prueba).ToString();
                label_comando_key.Text = "avanza";

                break;

                case 'a':
                comando[0] = Convert.ToByte(97);//cabecera 'a'
                port.Write(comando, 0, comando.Length);

                numero_prueba++;
                label_prueba.Text = (numero_prueba).ToString();
                label_comando_key.Text = "izquierda";
                break;

                case 's':
                comando[0] = Convert.ToByte(115);//cabecera 's'
                port.Write(comando, 0, comando.Length);

                numero_prueba++;
                label_prueba.Text = (numero_prueba).ToString();
                label_comando_key.Text = "atrás";
                break;

                case 'd':
                comando[0] = Convert.ToByte(100);//cabecera 'd'
                port.Write(comando, 0, comando.Length);

                numero_prueba++;
                label_prueba.Text = (numero_prueba).ToString();
                label_comando_key.Text = "derecha";
                break;

                case 'q':
                comando[0] = Convert.ToByte(113);//cabecera 'q'
                port.Write(comando, 0, comando.Length);

                numero_prueba++;
                label_prueba.Text = (numero_prueba).ToString();
                label_comando_key.Text = "detenerse";
                break;



            }

            
  
        }


        private void cr_adelante_Click(object sender, EventArgs e)
        {
            //Envio de comando movimiento
            byte[] comando = new byte[1];
            comando[0] = Convert.ToByte(119);//cabecera 'w'
            port.Write(comando, 0, comando.Length);

        }

        private void cr_atras_Click(object sender, EventArgs e)
        {
            //Envio de comando movimiento
            byte[] comando = new byte[1];
            comando[0] = Convert.ToByte(115);//cabecera 's'
            port.Write(comando, 0, comando.Length);
        }

        private void cr_derecha_Click(object sender, EventArgs e)
        {
            //Envio de comando movimiento
            byte[] comando = new byte[1];
            comando[0] = Convert.ToByte(100);//cabecera 'd'
            port.Write(comando, 0, comando.Length);
        }

        private void cr_izquierda_Click(object sender, EventArgs e)
        {
            //Envio de comando movimiento
            byte[] comando = new byte[1];
            comando[0] = Convert.ToByte(97);//cabecera 'a'
            port.Write(comando, 0, comando.Length);
        }

        private void label3_Click(object sender, EventArgs e)
        {

        }

        private void panel14_Paint(object sender, PaintEventArgs e)
        {

        }

        private void trackBar2_MouseDown(object sender, MouseEventArgs e)
        {
            label_vel_bomba.Text = trackBar2.Value.ToString();
        }

        private void on_val_izq_Click(object sender, EventArgs e)
        {
            on_val_izq.BackColor = Color.Peru;
            off_val_izq.BackColor = Color.White;

            //Envio de comando 
            byte[] comando = new byte[1];
            comando[0] = Convert.ToByte(51);//cabecera '3'
            port.Write(comando, 0, comando.Length);
        }

        private void off_val_izq_Click(object sender, EventArgs e)
        {
            off_val_izq.BackColor = Color.Peru;
            on_val_izq.BackColor = Color.White;

            //Envio de comando 
            byte[] comando = new byte[1];
            comando[0] = Convert.ToByte(52);//cabecera '4'
            port.Write(comando, 0, comando.Length);
        }

        private void on_val_der_Click(object sender, EventArgs e)
        {
            on_val_der.BackColor = Color.Peru;
            off_val_der.BackColor = Color.White;

            //Envio de comando 
            byte[] comando = new byte[1];
            comando[0] = Convert.ToByte(53);//cabecera 5'
            port.Write(comando, 0, comando.Length);
        }

        private void off_val_der_Click(object sender, EventArgs e)
        {
            off_val_der.BackColor = Color.Peru;
            on_val_der.BackColor = Color.White;

            //Envio de comando 
            byte[] comando = new byte[1];
            comando[0] = Convert.ToByte(54);//cabecera '6'
            port.Write(comando, 0, comando.Length);
         
        }

        private void button10_Click(object sender, EventArgs e)
        {

        }

        private void pictureBox15_Click(object sender, EventArgs e)
        {

        }

        private void button13_Click(object sender, EventArgs e)
        {
            if (puertos_serial.SelectedIndex > -1)
            {
                MessageBox.Show(String.Format("You selected port '{0}'", puertos_serial.SelectedItem));
                Connect(puertos_serial.SelectedItem.ToString());
                port.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);

            }
            else
            {
                MessageBox.Show("Please select a port first");
            }
        }

        private void Connect(string portName)
        {

            port = new SerialPort(portName);
            if (!port.IsOpen)
            {
                port.BaudRate = Convert.ToInt32(puertos_baudios.Text);
                //Añadiendo evento por recepcion serial:
                this.textBox1.Text = "Hola";
                port.Parity = Parity.None;
                port.StopBits = StopBits.One;
                port.DataBits = 8;
                port.Handshake = Handshake.None;

                port.Open();
                port.DataReceived += DataReceivedHandler;

            }

            //port.Write("Hola estoy conectado");
        }


        private void DataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort sp = (SerialPort)sender;
            
            bytes = sp.BytesToRead;
            sp.Read(byte_buffer, 0, bytes);
            this.Invoke(new EventHandler(Mostrar_TR));

        }


        private void Mostrar_TR(object sender, EventArgs e)
        {
           // contador_serial_event++;
           // if (contador_serial_event % 2 != 0)
           // {
                if (byte_buffer[0] == 120)
                {
                    lata = byte_buffer[1];
                    latb = byte_buffer[2];
                    latc = byte_buffer[3];
                    latd = byte_buffer[4];
                    lona = byte_buffer[5];
                    lonb = byte_buffer[6];
                    lonc = byte_buffer[7];
                    lond = byte_buffer[8];
                    yaw_raw = byte_buffer[9];
                    obs = byte_buffer[10];
                    i_tot = byte_buffer[11];
                    v_tot = byte_buffer[12];
                    nivel_tank = byte_buffer[13];

                    latitud_actual = integracion_gps(lata,latb,latc,latd);
                    longitud_actual = integracion_gps(lona,lonb,lonc,lond);
                    yaw = yaw_raw*2;
                    //yaw = yaw_raw*360/255;
                    i_tot = i_tot;
                    v_tot = v_tot;

                    latitud_mostrar.Text = latitud_actual.ToString();
                    longitud_mostrar.Text = longitud_actual.ToString();
                    yaw_mostrar.Text = yaw.ToString();
                    //velocidad_mostrar.Text = byte_buffer[4].ToString();
                    //energia_mostrar.Text = byte_buffer[5].ToString();
                    voltaje_mostrar.Text = v_tot.ToString();
                    tanque_mostrar.Text = nivel_tank.ToString();
                    corriente_mostrar.Text = i_tot.ToString();
                    //trabajo_mostrar.Text = byte_buffer[9].ToString();
                    obstaculo_mostrar.Text = obs.ToString();

                    if (lata == 12 && latb == 2 && lona == 77 && lonb == 4) { 
                    webBrowser2.Document.InvokeScript("mostrarcoordenada", new string[] { latitud_actual.ToString(), longitud_actual.ToString() });
                    webBrowser1.Document.InvokeScript("mostrarcoordenada", new string[] { latitud_actual.ToString(), longitud_actual.ToString() });

                    }


                }
               
                else if(byte_buffer[0] == 121) // Trama de sensado
                {
                        hum_amb = byte_buffer[1];
                        temp_amb = byte_buffer[2];
                        rad_sol = byte_buffer[3];
                        hum_sue = byte_buffer[4];

                        hum_amb = hum_amb;
                        temp_amb = temp_amb;
                        rad_sol = rad_sol;
                        hum_sue = hum_sue;
                        pH_sue = pH_sue;

                        richTextBox1.Text += hum_sue.ToString() + " agua " + "\n";
                        richTextBox3.Text += rad_sol.ToString() + " radiacion " + "\n";
                        richTextBox2.Text += pH_sue.ToString() + " pH " + "\n";
                        richTextBox4.Text += temp_amb.ToString() + " grados " + "\n";
                        richTextBox5.Text += hum_amb.ToString() + " humedad " + "\n";

                        try
                        {
                            using (SqlConnection conn = new SqlConnection("server=TOSHIBA\\ARTURO;" +
                                                              "Trusted_Connection=yes;" +
                                                              "database=reporte; " +
                                                              "connection timeout=30"))
                            {
                                conn.Open();
                                using (SqlCommand cmd =
                                    new SqlCommand("INSERT INTO [robot_paracas].[dbo].[humedad_suelo] VALUES(@valor , @fecha)", conn))
                                {
                                    cmd.Parameters.AddWithValue("@valor", hum_sue.ToString());
                                    cmd.Parameters.AddWithValue("@fecha",  DateTime.Now.ToString());
                                    int rows = cmd.ExecuteNonQuery();

                                    //rows number of record got inserted
                                }

                                using (SqlCommand cmd =
                                    new SqlCommand("INSERT INTO [robot_paracas].[dbo].[radiacion_solar] VALUES(@valor , @fecha)", conn))
                                {
                                    cmd.Parameters.AddWithValue("@valor", rad_sol.ToString());
                                    cmd.Parameters.AddWithValue("@fecha", DateTime.Now.ToString());
                                    int rows = cmd.ExecuteNonQuery();

                                    //rows number of record got inserted
                                }

                                using (SqlCommand cmd =
                                  new SqlCommand("INSERT INTO [robot_paracas].[dbo].[ph_suelo] VALUES(@valor , @fecha)", conn))
                                {
                                    cmd.Parameters.AddWithValue("@valor", pH_sue.ToString());
                                    cmd.Parameters.AddWithValue("@fecha", DateTime.Now.ToString());
                                    int rows = cmd.ExecuteNonQuery();

                                    //rows number of record got inserted
                                }

                                using (SqlCommand cmd =
                               new SqlCommand("INSERT INTO [robot_paracas].[dbo].[temp_ambiental] VALUES(@valor , @fecha)", conn))
                                {
                                    cmd.Parameters.AddWithValue("@valor", temp_amb.ToString());
                                    cmd.Parameters.AddWithValue("@fecha", DateTime.Now.ToString());
                                    int rows = cmd.ExecuteNonQuery();

                                    //rows number of record got inserted
                                }

                                using (SqlCommand cmd =
                                        new SqlCommand("INSERT INTO [robot_paracas].[dbo].[humedad_ambiental] VALUES(@valor , @fecha)", conn))
                                {
                                    cmd.Parameters.AddWithValue("@valor", hum_amb.ToString());
                                    cmd.Parameters.AddWithValue("@fecha", DateTime.Now.ToString());
                                    int rows = cmd.ExecuteNonQuery();

                                    //rows number of record got inserted
                                }


                            }
                        }
                        catch (SqlException ex)
                        {
                            //Log exception
                            //Display Error message
                        }


             }

          //}
            
        }

        public double  integracion_gps(int a , int b , int c , int d){

          return -((float)a+(float)b/100.0000000000+(float)c/10000.0000000000+(float)d/1000000.0000000000);  
  
        }


        private void button14_Click(object sender, EventArgs e)
        {   
            //port.Write("Chau me desconecto");
            port.Close();
        }

        private void button16_Click(object sender, EventArgs e)
        {
            desintegrar.Text = "Desintegración respecto a N - E  \n";

            for (int j = 1; j <= contador; j++)
            {
                a1[j] = Convert.ToInt16(Math.Floor(latitudes[j] / 100));
                b1[j] = Convert.ToInt16(Math.Floor(latitudes[j] - a1[j] * 100));
                c1[j] = Convert.ToInt16(Math.Floor(latitudes[j] * 100 - (a1[j] * 10000 + b1[j] * 100)));
                d1[j] = Convert.ToInt16(Math.Floor(latitudes[j] * 10000 - (a1[j] * 1000000 + b1[j] * 10000 + c1[j] * 100)));

                a2[j] = Convert.ToInt16(Math.Floor(longitudes[j] / 100));
                b2[j] = Convert.ToInt16(Math.Floor(longitudes[j] - a2[j] * 100));
                c2[j] = Convert.ToInt16(Math.Floor(longitudes[j] * 100 - (a2[j] * 10000 + b2[j] * 100)));
                d2[j] = Convert.ToInt16(Math.Floor(longitudes[j] * 10000 - (a2[j] * 1000000 + b2[j] * 10000 + c2[j] * 100)));

                desintegrar.Text += "P" + j + ":  " + a1[j].ToString() + " " + b1[j].ToString() + " " + c1[j].ToString() + " " + d1[j].ToString() + "       " + a2[j].ToString() + " " + b2[j].ToString() + " " + c2[j].ToString() + " " + d2[j].ToString() + "\n";

                int jj = 8 * (j - 1);
                buffer[jj] = a1[j];
                buffer[jj + 1] = b1[j];
                buffer[jj + 2] = c1[j];
                buffer[jj + 3] = d1[j];
                buffer[jj + 4] = a2[j];
                buffer[jj + 5] = b2[j];
                buffer[jj + 6] = c2[j];
                buffer[jj + 7] = d2[j];
            }
        }

        private void button1_Click_2(object sender, EventArgs e)
        {
            int i = contador;

            //Envio de ruta a C1
            byte[] hola = new byte[1];
            hola[0] = Convert.ToByte(120);//cabecera 'x'
            port.Write(hola, 0, hola.Length);

            byte[] size = new byte[1];
            size[0] = Convert.ToByte((i * 8));
            port.Write(size, 0, size.Length);

            for (int k = 0; k <= (8 * i); k++)
            {
                buffer_byte[0] = Convert.ToByte(buffer[k]);//'ASCII letra "t";
                port.Write(buffer_byte, 0, buffer_byte.Length);
            }

            hola[0] = Convert.ToByte(121);//cola'y'
            port.Write(hola, 0, hola.Length);

        }



        public double mayor(double[] nums)
        {
            double mayor_num = 0;
            mayor_num = nums[0];

            for (int p = 0; p < nums.Length; p++)
            {
                if (nums[p] > mayor_num)
                {
                    mayor_num = nums[p];
                }
            }

            return mayor_num;

        }

        public double menor(double[] nums)
        {
            double menor_num;
            menor_num = nums[0];

            for (int p = 0; p < nums.Length; p++)
            {
                if (nums[p] < menor_num)
                {
                    menor_num = nums[p];
                }
            }

            return menor_num;

        }

        public void regresion(double[] x, double[] y)
        {
            double sumxy = 0, sumxsumy = 0, sumx2 = 0, sum2x = 0, sumy = 0, sumx = 0, nn = 0;

            for (int p = 0; p < x.Length; p++)
            {
                sumx += x[p];
                sumy += y[p];
                sumxy += x[p] * y[p];
                sumx2 += Math.Pow(x[p], 2);
            }

            sumxsumy = sumx * sumy;
            sum2x = Math.Pow(sumx, 2);
            nn = x.Length;

            pendiente = (nn * sumxy - sumxsumy) / (nn * sumx2 - sum2x);
            offset = (sumy - pendiente * sumx) / (nn);

        }

        public void lugar_geometrico()
        {

            for (int col = 0; col < num_columnas; col++)
            {
                for (int fil = 0; fil < 16; fil++)
                {
                    switch (col)
                    {
                        case 0: regresion(X1, Y1);
                            cc1[fil] = Y1[fil] - pendiente * X1[fil];
                            pendientes[col] = pendiente;
                            break;

                        case 1: regresion(X2, Y2);
                            cc2[fil] = Y2[fil] - pendiente * X2[fil];
                            pendientes[col] = pendiente;
                            break;

                        case 2: regresion(X3, Y3);
                            cc3[fil] = Y3[fil] - pendiente * X3[fil];
                            pendientes[col] = pendiente;
                            break;

                        case 3: regresion(X4, Y4);
                            cc4[fil] = Y4[fil] - pendiente * X4[fil];
                            pendientes[col] = pendiente;
                            break;

                        case 4: regresion(X5, Y5);
                            cc5[fil] = Y5[fil] - pendiente * X5[fil];
                            pendientes[col] = pendiente;
                            break;

                        case 5: regresion(X6, Y6);
                            cc6[fil] = Y6[fil] - pendiente * X6[fil];
                            pendientes[col] = pendiente;
                            break;

                        case 6: regresion(X7, Y7);
                            cc7[fil] = Y7[fil] - pendiente * X7[fil];
                            pendientes[col] = pendiente;
                            break;

                        case 7: regresion(X8, Y8);
                            cc8[fil] = Y8[fil] - pendiente * X8[fil];
                            pendientes[col] = pendiente;
                            break;
                    }

                }
                switch (col)
                {
                    case 0: 
                        Lons_extremas[4 * col] = X1[0];
                        Lons_extremas[4 * col + 1] = X1[15];
                        Lons_extremas[4 * col + 2] = X1[0];
                        Lons_extremas[4 * col + 3] = X1[15];

                        Lats_extremas[4 * col] = pendiente * X1[0] + mayor(cc1);
                        Lats_extremas[4 * col + 1] = pendiente * X1[15] + mayor(cc1);
                        Lats_extremas[4 * col + 2] = pendiente * X1[0] + menor(cc1);
                        Lats_extremas[4 * col + 3] = pendiente * X1[15] + menor(cc1);

                        break;

                    case 1:
                        Lons_extremas[4 * col] = X2[0];
                        Lons_extremas[4 * col + 1] = X2[15];
                        Lons_extremas[4 * col + 2] = X2[0];
                        Lons_extremas[4 * col + 3] = X2[15];

                        Lats_extremas[4 * col] = pendiente * X2[0] + mayor(cc2);
                        Lats_extremas[4 * col + 1] = pendiente * X2[15] + mayor(cc2);
                        Lats_extremas[4 * col + 2] = pendiente * X2[0] + menor(cc2);
                        Lats_extremas[4 * col + 3] = pendiente * X2[15] + menor(cc2);
                        break;

                    case 2: 
                        Lons_extremas[4 * col] = X3[0];
                        Lons_extremas[4 * col + 1] = X3[15];
                        Lons_extremas[4 * col + 2] = X3[0];
                        Lons_extremas[4 * col + 3] = X3[15];

                        Lats_extremas[4 * col] = pendiente * X3[0] + mayor(cc3);
                        Lats_extremas[4 * col + 1] = pendiente * X3[15] + mayor(cc3);
                        Lats_extremas[4 * col + 2] = pendiente * X3[0] + menor(cc3);
                        Lats_extremas[4 * col + 3] = pendiente * X3[15] + menor(cc3);
                        break;

                    case 3:
                        Lons_extremas[4 * col] = X4[0];
                        Lons_extremas[4 * col + 1] = X4[15];
                        Lons_extremas[4 * col + 2] = X4[0];
                        Lons_extremas[4 * col + 3] = X4[15];

                        Lats_extremas[4 * col] = pendiente * X4[0] + mayor(cc4);
                        Lats_extremas[4 * col + 1] = pendiente * X4[15] + mayor(cc4);
                        Lats_extremas[4 * col + 2] = pendiente * X4[0] + menor(cc4);
                        Lats_extremas[4 * col + 3] = pendiente * X4[15] + menor(cc4);
                        break;

                    case 4:
                        Lons_extremas[4 * col] = X5[0];
                        Lons_extremas[4 * col + 1] = X5[15];
                        Lons_extremas[4 * col + 2] = X5[0];
                        Lons_extremas[4 * col + 3] = X5[15];

                        Lats_extremas[4 * col] = pendiente * X5[0] + mayor(cc5);
                        Lats_extremas[4 * col + 1] = pendiente * X5[15] + mayor(cc5);
                        Lats_extremas[4 * col + 2] = pendiente * X5[0] + menor(cc5);
                        Lats_extremas[4 * col + 3] = pendiente * X5[15] + menor(cc5);
                        break;

                    case 5:
                        Lons_extremas[4 * col] = X6[0];
                        Lons_extremas[4 * col + 1] = X6[15];
                        Lons_extremas[4 * col + 2] = X6[0];
                        Lons_extremas[4 * col + 3] = X6[15];

                        Lats_extremas[4 * col] = pendiente * X6[0] + mayor(cc6);
                        Lats_extremas[4 * col + 1] = pendiente * X6[15] + mayor(cc6);
                        Lats_extremas[4 * col + 2] = pendiente * X6[0] + menor(cc6);
                        Lats_extremas[4 * col + 3] = pendiente * X6[15] + menor(cc6);
                        break;

                    case 6:
                        Lons_extremas[4 * col] = X7[0];
                        Lons_extremas[4 * col + 1] = X7[15];
                        Lons_extremas[4 * col + 2] = X7[0];
                        Lons_extremas[4 * col + 3] = X7[15];

                        Lats_extremas[4 * col] = pendiente * X7[0] + mayor(cc7);
                        Lats_extremas[4 * col + 1] = pendiente * X7[15] + mayor(cc7);
                        Lats_extremas[4 * col + 2] = pendiente * X7[0] + menor(cc7);
                        Lats_extremas[4 * col + 3] = pendiente * X7[15] + menor(cc7);
                        break;

                    case 7:
                        Lons_extremas[4 * col] = X8[0];
                        Lons_extremas[4 * col + 1] = X8[15];
                        Lons_extremas[4 * col + 2] = X8[0];
                        Lons_extremas[4 * col + 3] = X8[15];

                        Lats_extremas[4 * col] = pendiente * X8[0] + mayor(cc8);
                        Lats_extremas[4 * col + 1] = pendiente * X8[15] + mayor(cc8);
                        Lats_extremas[4 * col + 2] = pendiente * X8[0] + menor(cc8);
                        Lats_extremas[4 * col + 3] = pendiente * X8[15] + menor(cc8);
                        break;
                }
            
            }

        }

        public void dibujo_limites()
        {

            for (int j = 0; j < (4 * num_columnas - 1); j += 2)
            {
                webBrowser2.Document.InvokeScript("dibujarlimites", new string[] { Lats_extremas[j].ToString(), Lats_extremas[j + 1].ToString(), Lons_extremas[j].ToString(), Lons_extremas[j + 1].ToString() });

            }
        }

        private void button17_Click(object sender, EventArgs e)
        {
            dibujo_limites();
        }

        private void button18_Click(object sender, EventArgs e)
        {

            for (int col = 0; col < (num_columnas - 1); col++)
            {
                double xcorner, ycorner;
                for (int fil = 0; fil < 15; fil++)
                {
                    switch (col)
                    {
                        case 0:
                            Xcorners[fil, col] = (X1[fil] + X1[fil + 1] + X2[fil] + X2[fil + 1]) / 4;
                            Ycorners[fil, col] = (Y1[fil] + Y1[fil + 1] + Y2[fil] + Y2[fil + 1]) / 4;
                            break;

                        case 1:
                            Xcorners[fil, col] = (X2[fil] + X2[fil + 1] + X3[fil] + X3[fil + 1]) / 4;
                            Ycorners[fil, col] = (Y2[fil] + Y2[fil + 1] + Y3[fil] + Y3[fil + 1]) / 4;
                            break;
                        case 2:
                            Xcorners[fil, col] = (X3[fil] + X3[fil + 1] + X4[fil] + X4[fil + 1]) / 4;
                            Ycorners[fil, col] = (Y3[fil] + Y3[fil + 1] + Y4[fil] + Y4[fil + 1]) / 4;
                            break;
                        case 3:
                            Xcorners[fil, col] = (X4[fil] + X4[fil + 1] + X5[fil] + X5[fil + 1]) / 4;
                            Ycorners[fil, col] = (Y4[fil] + Y4[fil + 1] + Y5[fil] + Y5[fil + 1]) / 4;
                            break;
                        case 4:
                            Xcorners[fil, col] = (X5[fil] + X5[fil + 1] + X6[fil] + X6[fil + 1]) / 4;
                            Ycorners[fil, col] = (Y5[fil] + Y5[fil + 1] + Y6[fil] + Y6[fil + 1]) / 4;
                            break;
                        case 5:
                            Xcorners[fil, col] = (X6[fil] + X6[fil + 1] + X7[fil] + X7[fil + 1]) / 4;
                            Ycorners[fil, col] = (Y6[fil] + Y6[fil + 1] + Y7[fil] + Y7[fil + 1]) / 4;
                            break;
                        case 6:
                            Xcorners[fil, col] = (X7[fil] + X7[fil + 1] + X8[fil] + X8[fil + 1]) / 4;
                            Ycorners[fil, col] = (Y7[fil] + Y7[fil + 1] + Y8[fil] + Y8[fil + 1]) / 4;
                            break;

                    }

                    xcorner = Xcorners[fil, col];
                    ycorner = Ycorners[fil, col];

                    webBrowser2.Document.InvokeScript("dibujarcorners", new string[] { ycorner.ToString(), xcorner.ToString() });

                }


            }

        }

        void mapeo() {

            //Primera columna
            X1[0] = -76.1248489778856; //Longitud
            X1[1] = -76.1248310870019;
            X1[2] = -76.1248168281736;
            X1[3] = -76.1248031799866;
            X1[4] = -76.124792783802;
            X1[5] = -76.1247829701431;
            X1[6] = -76.1247691100587;
            X1[7] = -76.1247542929933;
            X1[8] = -76.1247461974027;
            X1[9] = -76.1247316721397;
            X1[10] = -76.1247217679529;
            X1[11] = -76.1247013487762;
            X1[12] = -76.1246911863044;
            X1[13] = -76.1246784883618;
            X1[14] = -76.1246647478576;
            X1[15] = -76.1246489613976;


            Y1[0] = -13.8961777599162; // Latitud
            Y1[1] = -13.8961319708481;
            Y1[2] = -13.8960769158558;
            Y1[3] = -13.8960251983161;
            Y1[4] = -13.8959758992954;
            Y1[5] = -13.8959225726444;
            Y1[6] = -13.8958639948819;
            Y1[7] = -13.8958162120693;
            Y1[8] = -13.895764802789;
            Y1[9] = -13.8957101713157;
            Y1[10] = -13.8956561854041;
            Y1[11] = -13.8956058335753;
            Y1[12] = -13.8955555944771;
            Y1[13] = -13.8955028736761;
            Y1[14] = -13.8954507193811;
            Y1[15] = -13.8953933913278;

            //Segunda columna

            X2[0] = -76.1247947767646;
            X2[1] = -76.1247844426053;
            X2[2] = -76.1247726106520;
            X2[3] = -76.1247616048375;
            X2[4] = -76.1247429139265;
            X2[5] = -76.1247319841718;
            X2[6] = -76.1247172088627;
            X2[7] = -76.1247072974156;
            X2[8] = -76.1246945743569;
            X2[9] = -76.1246831099049;
            X2[10] = -76.1246702071180;
            X2[11] = -76.1246593611602;
            X2[12] = -76.1246466939859;
            X2[13] = -76.1246287973444;
            X2[14] = -76.1246125209164;
            X2[15] = -76.1246054776731;

            Y2[0] = -13.8961956875676;
            Y2[1] = -13.8961428153809;
            Y2[2] = -13.8960928814972;
            Y2[3] = -13.8960345035189;
            Y2[4] = -13.8959848037581;
            Y2[5] = -13.8959335398049;
            Y2[6] = -13.8958801629467;
            Y2[7] = -13.8958371003971;
            Y2[8] = -13.8957801962479;
            Y2[9] = -13.8957245631971;
            Y2[10] = -13.8956750678609;
            Y2[11] = -13.8956269097358;
            Y2[12] = -13.8955767626306;
            Y2[13] = -13.8955216189910;
            Y2[14] = -13.8954705340604;
            Y2[15] = -13.8954121159893;

            //Columna 3

            X3[0] = -76.1247454050457;
            X3[1] = -76.1247336459097;
            X3[2] = -76.1247228502061;
            X3[3] = -76.1247021558369;
            X3[4] = -76.1246869268644;
            X3[5] = -76.1246692399681;
            X3[6] = -76.1246634016042;
            X3[7] = -76.1246483717569;
            X3[8] = -76.1246334054016;
            X3[9] = -76.1246225402212;
            X3[10] = -76.1246080091145;
            X3[11] = -76.1245972734642;
            X3[12] = -76.1245856011693;
            X3[13] = -76.1245708755623;
            X3[14] = -76.1245597071439;
            X3[15] = -76.1245424976104;

            Y3[0] = -13.8962094624852;
            Y3[1] = -13.8961598824821;
            Y3[2] = -13.8961063500241;
            Y3[3] = -13.8960476279406;
            Y3[4] = -13.8959972477046;
            Y3[5] = -13.8959449004907;
            Y3[6] = -13.8958959546358;
            Y3[7] = -13.8958422222216;
            Y3[8] = -13.8957942611696;
            Y3[9] = -13.8957424965586;
            Y3[10] = -13.8956854091251;
            Y3[11] = -13.8956433469938;
            Y3[12] = -13.8955871590577;
            Y3[13] = -13.8955367250335;
            Y3[14] = -13.8954827120709;
            Y3[15] = -13.8954307950048;

            //Columna 4
            X4[0] = -76.1246897745092;
            X4[1] = -76.1246790876415;
            X4[2] = -76.1246627004883;
            X4[3] = -76.1246508525195;
            X4[4] = -76.1246380165655;
            X4[5] = -76.1246245769549;
            X4[6] = -76.1246141568552;
            X4[7] = -76.1245997699348;
            X4[8] = -76.1245887211940;
            X4[9] = -76.1245782652440;
            X4[10] = -76.1245615793062;
            X4[11] = -76.1245510983503;
            X4[12] = -76.1245352771739;
            X4[13] = -76.1245222173201;
            X4[14] = -76.1245129889459;
            X4[15] = -76.1244982631921;

            Y4[0] = -13.8962242291551;
            Y4[1] = -13.8961734567202;
            Y4[2] = -13.8961238273361;
            Y4[3] = -13.8960596722347;
            Y4[4] = -13.8960124404823;
            Y4[5] = -13.8959593155286;
            Y4[6] = -13.8959067722628;
            Y4[7] = -13.8958571072931;
            Y4[8] = -13.8958071900614;
            Y4[9] = -13.8957525428542;
            Y4[10] = -13.8956965135343;
            Y4[11] = -13.8956533709003;
            Y4[12] = -13.8955938847149;
            Y4[13] = -13.8955399436902;
            Y4[14] = -13.8954954814697;
            Y4[15] = -13.8954424611032;

            //columna 5
            X5[0] = -76.1246269683139;
            X5[1] = -76.1246160899683;
            X5[2] = -76.1246099157252;
            X5[3] = -76.1245955489782;
            X5[4] = -76.1245840319222;
            X5[5] = -76.1245667619525;
            X5[6] = -76.1245559610022;
            X5[7] = -76.1245386531463;
            X5[8] = -76.1245271938445;
            X5[9] = -76.1245194767342;
            X5[10] = -76.1245057524749;
            X5[11] = -76.1244928525706;
            X5[12] = -76.1244817525225;
            X5[13] = -76.1244671659023;
            X5[14] = -76.1244548864588;
            X5[15] = -76.1244455417438;

            Y5[0] = -13.8962319290479;
            Y5[1] = -13.8961865240866;
            Y5[2] = -13.8961386917371;
            Y5[3] = -13.8960763837195;
            Y5[4] = -13.8960220702575;
            Y5[5] = -13.8959783917542;
            Y5[6] = -13.8959274975580;
            Y5[7] = -13.8958648254387;
            Y5[8] = -13.8958199890906;
            Y5[9] = -13.8957694451073;
            Y5[10] = -13.8957170200874;
            Y5[11] = -13.8956681939011;
            Y5[12] = -13.8956155319214;
            Y5[13] = -13.8955594098713;
            Y5[14] = -13.8955142859982;
            Y5[15] = -13.8954633187895;

            //columna 6
            X6[0] = -76.1245749927973;
            X6[1] = -76.1245630023690;
            X6[2] = -76.1245535684760;
            X6[3] = -76.1245400263470;
            X6[4] = -76.1245276363282;
            X6[5] = -76.1245171866553;
            X6[6] = -76.1245034403135;
            X6[7] = -76.1244906813625;
            X6[8] = -76.1244785702181;
            X6[9] = -76.1244654823848;
            X6[10] = -76.1244520581457;
            X6[11] = -76.1244409826329;
            X6[12] = -76.1244332777151;
            X6[13] = -76.1244192974040;
            X6[14] = -76.1244075133320;
            X6[15] = -76.1243974938382;

            Y6[0] = -13.8962528415138;
            Y6[1] = -13.8961976203890;
            Y6[2] = -13.8961515397977;
            Y6[3] = -13.8960976951252;
            Y6[4] = -13.8960447002404;
            Y6[5] = -13.8959960067671;
            Y6[6] = -13.8959419023472;
            Y6[7] = -13.8958862888410;
            Y6[8] = -13.8958294549337;
            Y6[9] = -13.8957812820486;
            Y6[10] = -13.8957296433621;
            Y6[11] = -13.8956794591661;
            Y6[12] = -13.8956252835586;
            Y6[13] = -13.8955791234183;
            Y6[14] = -13.8955281820220;
            Y6[15] = -13.8954784192360;

            //columna7
            X7[0] = -76.1245276203144;
            X7[1] = -76.1245167084963;
            X7[2] = -76.1245059054661;
            X7[3] = -76.1244906837296;
            X7[4] = -76.1244764478693;
            X7[5] = -76.1244655647886;
            X7[6] = -76.1244544514908;
            X7[7] = -76.1244413154206;
            X7[8] = -76.1244273633245;
            X7[9] = -76.1244223456052;
            X7[10] = -76.1244079672916;
            X7[11] = -76.1243917238998;
            X7[12] = -76.1243793094015;
            X7[13] = -76.1243655177801;
            X7[14] = -76.1243561005467;
            X7[15] = -76.1243419701927;

            Y7[0] = -13.8962596761676;
            Y7[1] = -13.8962145506727;
            Y7[2] = -13.8961575501540;
            Y7[3] = -13.8961058960997;
            Y7[4] = -13.8960624959425;
            Y7[5] = -13.8960035574790;
            Y7[6] = -13.8959549571669;
            Y7[7] = -13.8958973815990;
            Y7[8] = -13.8958476897532;
            Y7[9] = -13.8957969458980;
            Y7[10] = -13.8957427713980;
            Y7[11] = -13.8956934259143;
            Y7[12] = -13.8956427342412;
            Y7[13] = -13.8955914872745;
            Y7[14] = -13.8955382282444;
            Y7[15] = -13.8954872202350;

            //columna8
            X8[0] = -76.1244734836072;
            X8[1] = -76.1244627622442;
            X8[2] = -76.1244530590199;
            X8[3] = -76.1244380632334;
            X8[4] = -76.1244291989380;
            X8[5] = -76.1244163259628;
            X8[6] = -76.1244004348947;
            X8[7] = -76.1243851087825;
            X8[8] = -76.1243738039715;
            X8[9] = -76.1243607361262;
            X8[10] = -76.1243484410520;
            X8[11] = -76.1243322465007;
            X8[12] = -76.1243174370831;
            X8[13] = -76.1243049006365;
            X8[14] = -76.1242998051463;
            X8[15] = -76.1242798443151;

            Y8[0] = -13.8962800238704;
            Y8[1] = -13.8962278759033;
            Y8[2] = -13.8961798911765;
            Y8[3] = -13.8961289059266;
            Y8[4] = -13.8960761079627;
            Y8[5] = -13.8960249636787;
            Y8[6] = -13.8959751222262;
            Y8[7] = -13.8959173600943;
            Y8[8] = -13.8958660481126;
            Y8[9] = -13.8958159041286;
            Y8[10] = -13.8957592123593;
            Y8[11] = -13.8957101564345;
            Y8[12] = -13.8956453026223;
            Y8[13] = -13.8955972010945;
            Y8[14] = -13.8955462668998;
            Y8[15] = -13.8954990230945;

        }

        private void button19_Click(object sender, EventArgs e)
        {
            //Envio de ruta a C1
            byte[] hola = new byte[1];
            //------------------------------------------------------------------
            //Envio de tarea a C2
            hola[0] = Convert.ToByte(122);//cabecera 'z'
            port.Write(hola, 0, hola.Length);


            hola[0] = Convert.ToByte(int.Parse(comboBox1.SelectedIndex.ToString()));
            port.Write(hola, 0, hola.Length);
            hola[0] = Convert.ToByte(textBox1.Text);
            port.Write(hola, 0, hola.Length);
            hola[0] = Convert.ToByte(textBox2.Text);
            port.Write(hola, 0, hola.Length);
            hola[0] = Convert.ToByte(textBox3.Text);
            port.Write(hola, 0, hola.Length);
            hola[0] = Convert.ToByte(textBox4.Text);
            port.Write(hola, 0, hola.Length);
            hola[0] = Convert.ToByte(textBox5.Text);
            port.Write(hola, 0, hola.Length);
            hola[0] = Convert.ToByte(textBox6.Text);
            port.Write(hola, 0, hola.Length);
            hola[0] = Convert.ToByte(textBox7.Text);
            port.Write(hola, 0, hola.Length);


            hola[0] = Convert.ToByte(102);//cola'f'
            port.Write(hola, 0, hola.Length);
            //------------------------------------------------------------------

        }


        private void button3_Click(object sender, EventArgs e)
        {
           /* string consulta = " SELECT [año] , [apellido] FROM [reporte].[dbo].[familia] ";

            SqlConnection myConnection = new SqlConnection("server=TOSHIBA\\ARTURO;" +
                                                   "Trusted_Connection=yes;" +
                                                   "database=reporte; " +
                                                   "connection timeout=30");*/

            int k  = int.Parse(variable_elegida.SelectedIndex.ToString());
            string nombre_variable = "humedad_suelo";

            switch (k)
            {
                case 0:
                    nombre_variable = "humedad_suelo";
                    break;
                case 1:
                    nombre_variable = "ph_suelo";
                    break;
                case 2:
                    nombre_variable = "radiacion_solar";
                    break;
                case 3:
                    nombre_variable = "temp_ambiental";
                    break;
                case 4:
                    nombre_variable = "humedad_ambiental";
                    break;
                case 5:
                    nombre_variable = "otro";
                    break;

            }

            string consulta = " SELECT [valor] , [fecha] FROM [robot_paracas].[dbo].[" + nombre_variable +  "] ";

            SqlConnection myConnection = new SqlConnection("server=TOSHIBA\\ARTURO;" +
                                                   "Trusted_Connection=yes;" +
                                                   "database=reporte; " +
                                                   "connection timeout=30");

            myConnection.Open();
            SqlCommand cmd = new SqlCommand(consulta, myConnection);
            SqlDataAdapter da = new SqlDataAdapter(cmd);
            DataTable dt = new DataTable();
            da.Fill(dt);
            dataGridView2.DataSource = dt;
            myConnection.Close();


           
        }

        private void panel1_Paint(object sender, PaintEventArgs e)
        {

        }

        private void bomba_state_Click(object sender, EventArgs e)
        {
            bomba_ON.BackColor = Color.Peru;
            BOMBA_OFF.BackColor = Color.White;

            //Envio de comando 
            byte[] comando = new byte[1];
            comando[0] = Convert.ToByte(49);//cabecera '1'
            port.Write(comando, 0, comando.Length);

        }

        private void BOMBA_OFF_Click(object sender, EventArgs e)
        {
            BOMBA_OFF.BackColor = Color.Peru;
            bomba_ON.BackColor = Color.White;

            //Envio de comando 
            byte[] comando = new byte[1];
            comando[0] = Convert.ToByte(50);//cabecera '2'
            port.Write(comando, 0, comando.Length);
        }

        private void send_phid_Click(object sender, EventArgs e)
        {
            //Conversion a 2 partes el phi deseado
            int angulo_ref= int.Parse(phid.Text);
            int phid_p1,phid_p2;

            if (angulo_ref >= 360)
            {
                angulo_ref = 360;
            }

            if (angulo_ref <= 0)
            {
                angulo_ref = 0;
            }

            if (angulo_ref > 255)
            {
                phid_p1 = 255;
                phid_p2 = angulo_ref - 255;
            }
            else
            {
                phid_p1 = angulo_ref;
                phid_p2 = 0;

            }

            //Envio de comando 
            byte[] comando = new byte[1];
            comando[0] = Convert.ToByte(112);//cabecera 'p' de la palabra phi deseado
            port.Write(comando, 0, comando.Length); //Valor de phi deseado
            //comando[0] = Convert.ToByte(phid.Text);
            comando[0] = Convert.ToByte(phid_p1);
            port.Write(comando, 0, comando.Length);
            comando[0] = Convert.ToByte(phid_p2); 
            port.Write(comando, 0, comando.Length);
            comando[0] = 0; //Velocidad lineal de prueba
            port.Write(comando, 0, comando.Length);
        }

        private void BORRAR_COORDENADAS_Click(object sender, EventArgs e)
        {
            //Borrando richtextboxs
            desintegrar.Text = "";
            monitor.Text = "";

            //Resetear VALORES
            //En el CS
            for (int i = 0; i < contador; i++)
            {
                latitudes[i] = 0;
                longitudes[i] = 0;
            }
            contador = 0;

            //En el JS
            webBrowser2.Document.InvokeScript("borrar_coordenadas", new string[] { });


        }

        private void webBrowser2_DocumentCompleted(object sender, WebBrowserDocumentCompletedEventArgs e)
        {

        }

        private void button19_Click_1(object sender, EventArgs e)
        {
            webBrowser1.Document.InvokeScript("borrar_coordenadas", new string[] { });

        }


        private void richTextBox8_KeyPress(object sender, KeyPressEventArgs e)
        {
            byte[] comando = new byte[1];

            switch (e.KeyChar)
            {

                case 'w':
                    comando[0] = Convert.ToByte(119);//cabecera 'w'
                    port.Write(comando, 0, comando.Length);

                    numero_prueba++;
                    label_prueba.Text = (numero_prueba).ToString();
                    label_comando_key.Text = "avanza";

                    break;

                case 'a':
                    comando[0] = Convert.ToByte(97);//cabecera 'a'
                    port.Write(comando, 0, comando.Length);

                    numero_prueba++;
                    label_prueba.Text = (numero_prueba).ToString();
                    label_comando_key.Text = "izquierda";
                    break;

                case 's':
                    comando[0] = Convert.ToByte(115);//cabecera 's'
                    port.Write(comando, 0, comando.Length);

                    numero_prueba++;
                    label_prueba.Text = (numero_prueba).ToString();
                    label_comando_key.Text = "atrás";
                    break;

                case 'd':
                    comando[0] = Convert.ToByte(100);//cabecera 'd'
                    port.Write(comando, 0, comando.Length);

                    numero_prueba++;
                    label_prueba.Text = (numero_prueba).ToString();
                    label_comando_key.Text = "derecha";
                    break;

                case 'q':
                    comando[0] = Convert.ToByte(113);//cabecera 'q'
                    port.Write(comando, 0, comando.Length);

                    numero_prueba++;
                    label_prueba.Text = (numero_prueba).ToString();
                    label_comando_key.Text = "detenerse";
                    break;



            }
        }

        private void richTextBox10_KeyPress(object sender, KeyPressEventArgs e)
        {
            byte[] comando = new byte[1];

            switch (e.KeyChar)
            {

                case 'w':
                    comando[0] = Convert.ToByte(119);//cabecera 'w'
                    port.Write(comando, 0, comando.Length);

                    numero_prueba++;
                    label_prueba.Text = (numero_prueba).ToString();
                    label_comando_key.Text = "avanza";

                    break;

                case 'a':
                    comando[0] = Convert.ToByte(97);//cabecera 'a'
                    port.Write(comando, 0, comando.Length);

                    numero_prueba++;
                    label_prueba.Text = (numero_prueba).ToString();
                    label_comando_key.Text = "izquierda";
                    break;

                case 's':
                    comando[0] = Convert.ToByte(115);//cabecera 's'
                    port.Write(comando, 0, comando.Length);

                    numero_prueba++;
                    label_prueba.Text = (numero_prueba).ToString();
                    label_comando_key.Text = "atrás";
                    break;

                case 'd':
                    comando[0] = Convert.ToByte(100);//cabecera 'd'
                    port.Write(comando, 0, comando.Length);

                    numero_prueba++;
                    label_prueba.Text = (numero_prueba).ToString();
                    label_comando_key.Text = "derecha";
                    break;

                case 'q':
                    comando[0] = Convert.ToByte(113);//cabecera 'q'
                    port.Write(comando, 0, comando.Length);

                    numero_prueba++;
                    label_prueba.Text = (numero_prueba).ToString();
                    label_comando_key.Text = "detenerse";
                    break;

            }
        }

        private void richTextBox11_KeyPress(object sender, KeyPressEventArgs e)
        {
            byte[] comando = new byte[1];

            switch (e.KeyChar)
            {

                case 'w':
                    comando[0] = Convert.ToByte(119);//cabecera 'w'
                    port.Write(comando, 0, comando.Length);

                    numero_prueba++;
                    label_prueba.Text = (numero_prueba).ToString();
                    label_comando_key.Text = "avanza";

                    break;

                case 'a':
                    comando[0] = Convert.ToByte(97);//cabecera 'a'
                    port.Write(comando, 0, comando.Length);

                    numero_prueba++;
                    label_prueba.Text = (numero_prueba).ToString();
                    label_comando_key.Text = "izquierda";
                    break;

                case 's':
                    comando[0] = Convert.ToByte(115);//cabecera 's'
                    port.Write(comando, 0, comando.Length);

                    numero_prueba++;
                    label_prueba.Text = (numero_prueba).ToString();
                    label_comando_key.Text = "atrás";
                    break;

                case 'd':
                    comando[0] = Convert.ToByte(100);//cabecera 'd'
                    port.Write(comando, 0, comando.Length);

                    numero_prueba++;
                    label_prueba.Text = (numero_prueba).ToString();
                    label_comando_key.Text = "derecha";
                    break;

                case 'q':
                    comando[0] = Convert.ToByte(113);//cabecera 'q'
                    port.Write(comando, 0, comando.Length);

                    numero_prueba++;
                    label_prueba.Text = (numero_prueba).ToString();
                    label_comando_key.Text = "detenerse";
                    break;

            }
        }

        private void label20_Click(object sender, EventArgs e)
        {

        }


    }
}
