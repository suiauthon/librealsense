// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 FRAMOS GmbH.

namespace Intel.RealSense
{
    public static class D400e
    {
        public static double GetHeartbeatTime()
        {
            object err;
            return NativeMethods.rs2_d400e_get_heartbeat_time(out err);
        }

        public static void SetHeartbeatTime(double time)
        {
            object err;
            NativeMethods.rs2_d400e_set_heartbeat_time(time, out err);
        }

        public static int GetBufferCount()
        {
            object err;
            return NativeMethods.rs2_d400e_get_buffer_count(out err);
        }

        public static void SetBufferCount(int bufferCount)
        {
            object err;
            NativeMethods.rs2_d400e_set_buffer_count(bufferCount, out err);
        }

        public static IntPtr create_cs_camera_config()
        {
            object err;
            return NativeMethods.rs2_d400e_create_cs_camera_config(out err);
        }

        public static void add_ip_to_cs_camera_config(IntPtr cs_config, string ip)
        {
            object err;
            byte[] cstr = Encoding.ASCII.GetBytes(ip);
            NativeMethods.rs2_d400e_add_ip_to_cs_camera_config(cs_config, cstr, out err);
        }

        public static void add_sn_to_cs_camera_config(IntPtr cs_config, string sn)
        {
            object err;
            byte[] cstr = Encoding.ASCII.GetBytes(ip);
            NativeMethods.rs2_d400e_add_sn_to_cs_camera_config(cs_config, cstr, out err);
        }
    }
}
