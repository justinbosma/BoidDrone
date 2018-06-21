
(
SynthDef(\pmOsc, {|carfreq = 440, modfreq = 1, pmindex = 1, modphase = 1, amp = 1, out = 0|

	var sig = PMOsc.ar(carfreq: carfreq, modfreq: modfreq, pmindex: pmindex, modphase: modphase, mul: amp);
    Out.ar(out, sig);
}).add;

SynthDef(\fbOsc, {|freq = 440, feedback = 0, amp = 1, out = 0|
	var sig = SinOscFB.ar(freq: freq, feedback: feedback, mul: amp);
    Out.ar(out, sig);
}).add;

SynthDef(\sin, {|freq = 440, rez = 0, amp = 1, out = 0|
	var sig = Mix.ar(SinOsc.ar(freq: [freq, freq/2], mul: amp*0.3));
	sig = BPF.ar(sig, freq: freq - 50, rq: rez, mul: amp);
	sig = Disintegrator.ar(sig, MouseX.kr(0, 1), MouseY.kr(0, 1));
	Out.ar(out, Pan2.ar(sig));
}).add;

~synths = Dictionary.new;


OSCdef(\makeSin, {|msg|
    msg.postln;
	if(msg[3] < 200, {
		x = Synth(\sin, [\freq: msg[2], \rez: msg[4], \amp: msg[3]]);
        ~synths.put(msg[1], x);
        //x.onFree({~total = ~total - 1; ~synths.removeAt(msg[6]);});
        //~total = ~total + 1;
        //~total.postln;
    }, {});
}, "/makeSin");

 OSCdef(\control, {|msg|
	~synths[msg[1]].set(\freq, msg[2], \rez, msg[4], \amp, msg[3]);
 }, "/control");
)


