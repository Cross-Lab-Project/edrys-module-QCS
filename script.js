edrys_mode = false
serial_onRead = null;

let connected = false;
let port = null

function log(...x){
    x = x.join("")
    const w = document.getElementById("serlog")
    w.append(x , "\n")
}


async function serial_connect(baudRate, userInitiated = false) {
    if (connected) return connected;

    if(!port && !userInitiated){ //automatic port selection (last used?)
        let ports =  await navigator.serial.getPorts();
        if ((ports) && (Array.isArray(ports)) && (ports.length > 0)) {
            port = ports[0];
        }
    }

    if (!port) {
        //manual port
        port = await navigator.serial.requestPort()
    }

    if (port && !connected) {
        await port.open({ baudRate: baudRate });
        connected = true
        setTimeout(e => serial_read(), 1)
    }

    return connected
}

let reader = null
let readableStreamClosed = null

async function serial_read() {
    if (!port) { connected = false; return; }
    reader = port.readable.getReader();
    while (true) {
        const { value, done } = await reader.read();
        if (done) {
            reader.releaseLock();
            break;
        }
        if(serial_onRead) serial_onRead(value)
    }
}

async function serial_disconnect() {
    if(port && connected) await port.close()
}


let writer = null
let writableStreamClosed = null
async function serial_write(text) {
    if (!port || !connected) {
        // no serial connection
        connected = false;
        com_send("dbg", "no serial:\t","text");
        return;
    }
    if (!writer) {
        const textEncoder = new TextEncoderStream();
        writableStreamClosed = textEncoder.readable.pipeTo(port.writable);
        writer = textEncoder.writable.getWriter();
    }
    if (writer)
        await writer.write(text);
}


interprete = (x)=>{ com_send("rsp",JSON.stringify(FrameDec(x)))}

// #define		TM_QUATERNION		0
// #define		TM_RPY				1
// #define		TM_GYRO				2
// #define		TM_GPS				3
// #define		TM_OF				4
// #define		TM_HEIGHT			5
// #define		TM_PID				6
// #define		TM_MESSAGES			7

// interprete = (s)=>{
//     d = s.slice(1,-1).split(',').map(Number)
//     // readid
//     id = d[0]
//     switch(id){
//         case 0:
//             log("frame:",s, "zero id")
//         default:
//             log("frame:",d, "unknown id", id)
//     }
// }


// #define FRAME_START '$'
// #define FRAME_END   '#'
// #define FRAME_START2 '+'
// #define FRAME_END2   '*'

function CstoU(x){
    let ret = 0
    const len = x.length;
    for( var i=0; i < len ; i++ ) {
        ret += x.charCodeAt(i) << (8 * (len - 1 -i))
    }
    return ret
}
textDec= new TextDecoder()
textEnc= new TextEncoder()

function FrameDec(x){
    //.slice(2,-2)// cut sof and eof
    let dv = new DataView(x.buffer) // get Dataview from buffer
    if(dv.byteLength < 6)
        return null;

    let ret = {head:[], data:[]}
    let i = 0;
    for(; i < 6; i++)
        ret.head.push(dv.getUint8(i));
    for(; i < dv.byteLength - 4; i += 4)
        ret.data.push(dv.getFloat32(i));
    return ret
}

frameReader = {
    frame: null ,
    no_frame : null ,
    last_c : null,

    init() {
        let sof = '$+';
        let eof = '#*';
        this.sof = textEnc.encode(sof);
        this.eof = textEnc.encode(eof);
    },


    // frame:  sof1 sof2 data eof1 eof2
    read(v){
        if (!this.sof) init();
        for (i in v) {
            let c = v[i];
            if( c == this.eof[1] && this.last_c == this.eof[0] && this.frame){ //deal with frame ends
                //frame complete
                let fbu8 = new Uint8Array( new ArrayBuffer(this.frame.length));
                fbu8.set(this.frame);
                let f
                try{
                f = FrameDec(fbu8);
                } catch({e, msg}) {
                    com_send("dbg",msg)
                    com_send("dbg",JSON.stringify(fbu8))
                }
                this.recvd_frame(JSON.stringify(FrameDec(fbu8)))

                this.frame = null;
                this.last_c = null;
            } else if( c == this.sof[1] && this.last_c == this.sof[0]){ //deal with frame starts
                //new frame
                if (this.frame){ //there is an incomplete frame in buffer
                    this.recvd_dbg("incomplete frame:".concat(JSON.stringify(this.frame)));
                }
                // TODO may or may not do the same for buffer messages
                // start an empty frame
                this.frame = [];
                this.last_c = null;
            } else {
                if (c == this.sof[0] || c == this.eof[0]){
                    this.last_c = c;
                } else {
                    if(this.frame) {
                        if(this.last_c) this.frame.push(this.last_c)
                        this.frame.push(c)
                    } else {
                        if (!this.no_frame ) this.no_frame = []
                        if (this.last_c) this.no_frame.push(this.last_c);
                        this.no_frame.push(c)
                        if(c == "\n".charCodeAt(0)){
                            // read msg bytewise (no UTF8) TODO this assumption has to be checked
                            let fbu8 = new Uint8Array( new ArrayBuffer(this.no_frame.length));
                            fbu8.set(this.no_frame)
                            this.recvd_msg(textDec.decode(fbu8))
                        }
                    }
                    this.last_c = null;
                }
            }
        }
    },

    recvd_frame(x){com_send("rsp",x)},

    recvd_msg(x){com_send("msg",x)},

    recvd_dbg(x){com_send("dbg",x)},
}

//q is a qcs specific kindof base64 using codes 46-57, 65-90, 97-123
function qtof(s) {
    let raw = 0
    for(i=5;i>=0;i--){
        //Einzelness zeichen nehmen
        let c= s.charCodeAt(i);

        // ASCII-Code in den Bereich 0-63 abbilden
        if (c <= 57){
                c -= 46;
        }else if(c <= 90){
                c -= 65-12;
        }else {
                c-= 97-(12+26);
        }
        // ergebniss 6 bit nach links shiften
        raw *= 64;
        //  aktuelle 6 bit vorne einsetzen
        raw += c;
    }
    // to float32
    let dv = new DataView(new ArrayBuffer(4));
    dv.setUint32(0,raw);
    return dv.getFloat32(0);
}


function ftoq(f){
    let dv = new DataView(new ArrayBuffer(4));
    //assume input is float
    dv.setFloat32(0,f)
    let raw_data = dv.getUint32(0);
    let s = ""
    for(i=0;i<6;i++){
        c= raw_data % 64;
        raw_data = raw_data / 64;

        if (c < 12){
            c += 46;
        }else if(c < 12+26){
            c += 65-12;
        }else {
            c+= 97-(12+26);
        }
        s += String.fromCharCode(c);
    }
    return s
}

//{ head:"" data:[]}
function enc_command(x){
    let frame = "$";
    frame += x.head; //head is string "143"
    frame += ":";
    //data is floats encoded to q
    for( i in x.data )
        frame += ftoq(x.data[i]) + ",";
    if (x.data.length == 0) return frame + "#";
    else return frame.slice(0,-1)+"#";
}

//decode comd s = s.split(",");for(i in s) s[i] = qtof(s[i]); s

function dec_command(frame){
    let f = frame;
    let r= {head:"",data:[]};
    if(!f[0] == "$" || !f[f.length-1] == "#") return {}
    f = f.slice(1,-1)
    f = f.split(":");
    r.head = f[0]
    f = f[1].split(",");
    for(i in f) r.data[i] = qtof(f[i]);
    return r
}

frameReader.init()
serial_onRead = (v)=>{frameReader.read(v)}

// create a tab separated JSON string (for printing)
JSTR = (x)=>{return JSON.stringify(x).replace(/,/ ,",\t");}

com_recieve = ({ from, subject, body }) => {
    if(subject == 'cmd'){
        let x = JSON.parse(body);
        if ((!edrys_mode || Edrys.role == 'station') && connected) serial_write(enc_command(x));
        else if(byId("log_cmd").checked) log("cmd:\t", JSTR(x));
    }
    if (subject== 'rsp'){
        let x = JSON.parse(body);
        if(byId("log_rsp").checked) log("rsp:\t", JSTR(x));
    }
    if (subject== 'msg'){
        // msg do not need JSON
        let x = (body);
        if(byId("log_msg").checked) log(x);
    }
    if (subject== 'dbg'){
        // dbg messages are not interpreted
        let x = body;
        if(byId("log_dbg").checked) log("dbg:\t", (x));
        }
}

com_send = (subject, ...body) => {
    body = "".concat(body.join(""))
    if(edrys_mode) Edrys.sendMessage(subject,body);
    else com_recieve ({from:"me", subject:subject, body:body});
}

//Edrys.onMessage(com_recieve)


// serial_connect(115200)
// 57600 << 19200 9600
// navigator.serial.requestPort()
/* egg samples
const imu = [
    0x24, 0x2b, 0x00, 0x27,
    0x02, 0x00, 0x02, 0x09,
    0x3c, 0xf9, 0x01, 0xc0,
    0x3d, 0x06, 0x5e, 0x68,
    0x3c, 0xc1, 0x6f, 0x3f,
    0x43, 0xd7, 0x82, 0x24,
    0xc1, 0x3f, 0x99, 0xa0,
    0xc4, 0x62, 0x1d, 0x46,
    0x43, 0xe0, 0x6d, 0x20,
    0xc1, 0x91, 0x82, 0x00,
    0xc4, 0x6a, 0x6f, 0xa0,
    0x23, 0x2a
]

const mag = [
    0x24, 0x2b, 0x00, 0x27,
    0x02, 0x00, 0x04, 0x09,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0xff, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x23, 0x2a
]

const  armsg =
[
    0x24, 0x2b, 0x00, 0x0f,
    0x02, 0x00, 0xc1, 0x03,
    0x41, 0x8e, 0xaf, 0xc0,
    0x40, 0x28, 0xbd, 0x00,
    0xc2, 0x05, 0x25, 0xa0,
    0x23, 0x2a, 0x53, 0x65,
    0x6e, 0x64, 0x20, 0x41,
    0x63, 0x63, 0x20, 0x42,
    0x69, 0x61, 0x73, 0x0a,
    0x24, 0x2b, 0x00, 0x0f,
    0x02, 0x00, 0xc1, 0x03,
    0x41, 0x8e, 0xaf, 0xc0,
    0x40, 0x28, 0xbd, 0x00,
    0xc2, 0x05, 0x25, 0xa0,
    0x23, 0x2a, 0x53, 0x65,
    0x6e, 0x64, 0x20, 0x41,
    0x63, 0x63, 0x20, 0x42,
    0x69, 0x61, 0x73, 0x0a
];

*/
