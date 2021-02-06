function A = g_Linear_A(in1,in2)
%G_LINEAR_A
%    A = G_LINEAR_A(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    08-Jun-2020 16:10:21

dq1 = in1(4,:);
dq2 = in1(5,:);
dq3 = in1(6,:);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
t2 = q2.*2.0;
t3 = q3.*2.0;
t4 = cos(t2);
t5 = t4.*8.1e1;
t6 = cos(t3);
t7 = t6.*4.5e1;
t8 = t2+t3;
t9 = cos(t8);
t28 = t9.*9.0;
t10 = t5+t7-t28-1.69e2;
t11 = 1.0./t10;
t12 = q1+t2;
t13 = sin(t12);
t14 = q1+t2+t3;
t15 = sin(t14);
t16 = dq1.^2;
t17 = q2-q3;
t18 = cos(t17);
t19 = dq2.^2;
t20 = q2+t3;
t21 = cos(t20);
t22 = sin(q2);
t23 = q2+q3;
t24 = cos(t23);
t25 = dq3.^2;
t26 = sin(t20);
t27 = cos(q2);
t29 = q1-t3;
t30 = q1+t3;
t31 = sin(t17);
t32 = sin(t2);
t33 = sin(t8);
t34 = sin(t23);
t35 = sin(t29);
t36 = sin(t30);
t37 = t15.*2.943e3;
t38 = sin(t3);
t39 = t16.*t18.*4.5e2;
t40 = t18.*t19.*4.5e2;
t41 = t18.*t25.*4.5e2;
t42 = t34.*u3.*2.1e3;
t43 = t16.*t24.*2.5e2;
t44 = t19.*t24.*2.5e2;
t45 = t24.*t25.*2.5e2;
t46 = dq1.*dq2.*t18.*9.0e2;
t47 = dq1.*dq3.*t18.*9.0e2;
t48 = dq2.*dq3.*t18.*9.0e2;
t49 = dq1.*dq2.*t24.*5.0e2;
t50 = dq1.*dq3.*t24.*5.0e2;
t51 = dq2.*dq3.*t24.*5.0e2;
t52 = 1.0./t10.^2;
t53 = u1.*2.3e3;
t54 = cos(t12);
t55 = t54.*1.32435e4;
t56 = cos(t29);
t57 = t56.*4.4145e3;
t58 = cos(t30);
t59 = t58.*4.4145e3;
t60 = cos(q1);
t61 = cos(t14);
t62 = t16.*t22.*2.25e3;
t63 = t19.*t22.*2.25e3;
t64 = t6.*u2.*9.0e2;
t65 = t16.*t31.*4.5e2;
t66 = t19.*t31.*4.5e2;
t67 = t25.*t31.*4.5e2;
t68 = t16.*t32.*1.35e3;
t69 = t27.*u3.*2.7e3;
t70 = t16.*t34.*2.5e2;
t71 = t19.*t34.*2.5e2;
t72 = t25.*t34.*2.5e2;
t73 = t21.*u2.*9.0e2;
t74 = t18.*u3.*2.7e3;
t75 = dq1.*dq2.*t22.*4.5e3;
t76 = dq1.*dq2.*t31.*9.0e2;
t77 = dq1.*dq3.*t31.*9.0e2;
t78 = dq2.*dq3.*t31.*9.0e2;
t79 = dq1.*dq2.*t34.*5.0e2;
t80 = dq1.*dq3.*t34.*5.0e2;
t81 = dq2.*dq3.*t34.*5.0e2;
t116 = t60.*3.1392e4;
t117 = t61.*1.4715e3;
t119 = t6.*u1.*9.0e2;
t121 = t19.*t26.*1.5e2;
t122 = t24.*u3.*2.1e3;
t123 = t16.*t33.*1.5e2;
t124 = t21.*u3.*9.0e2;
t125 = dq1.*dq2.*t26.*3.0e2;
t185 = t27.*u2.*2.7e3;
t82 = t53+t55+t57+t59+t62+t63+t64+t65+t66+t67+t68+t69+t70+t71+t72+t73+t74+t75+t76+t77+t78+t79+t80+t81-t116-t117-t119-t121-t122-t123-t124-t125-t185-u2.*2.3e3-t16.*t26.*1.5e2;
t83 = dq1.*t34.*5.0e2;
t84 = dq2.*t34.*5.0e2;
t85 = dq3.*t34.*5.0e2;
t86 = dq1.*t22.*4.5e3;
t87 = dq2.*t22.*4.5e3;
t88 = dq1.*t31.*9.0e2;
t89 = dq2.*t31.*9.0e2;
t90 = dq3.*t31.*9.0e2;
t91 = t13.*1.32435e4;
t92 = t35.*4.4145e3;
t93 = t36.*4.4145e3;
t94 = sin(q1);
t95 = q1-q2;
t96 = sin(t95);
t97 = t96.*2.6487e4;
t98 = q1+q2+t3;
t99 = sin(t98);
t100 = t99.*2.943e3;
t101 = q1+q2;
t102 = sin(t101);
t103 = -q1+q2+t3;
t104 = sin(t103);
t105 = t104.*4.4145e3;
t106 = q3+t2;
t107 = cos(t106);
t108 = t9.*t16.*3.0e2;
t109 = t26.*u2.*1.8e3;
t110 = t26.*u3.*9.0e2;
t111 = t19.*t27.*2.25e3;
t112 = dq1.*dq2.*t27.*4.5e3;
t113 = t32.*1.62e2;
t114 = t33.*1.8e1;
t115 = t113-t114;
t118 = sin(q3);
t120 = sin(t106);
t126 = t36.*8.829e3;
t127 = t38.*u2.*1.8e3;
t128 = t19.*t21.*3.0e2;
t129 = t33.*u3.*1.8e3;
t130 = cos(q3);
t131 = dq1.*dq2.*t21.*6.0e2;
t132 = t38.*9.0e1;
t133 = u3.*4.7e3;
t134 = cos(t95);
t135 = cos(t98);
t136 = cos(t101);
t137 = t136.*2.20725e4;
t138 = cos(t103);
t139 = t138.*4.4145e3;
t140 = t16.*t22.*7.2e3;
t141 = t16.*t120.*4.5e2;
t142 = t19.*t120.*4.5e2;
t143 = t25.*t120.*4.5e2;
t144 = t9.*u2.*9.0e2;
t145 = t16.*t32.*2.7e3;
t146 = t19.*t32.*1.35e3;
t147 = t27.*u1.*2.7e3;
t148 = t130.*u3.*5.7e3;
t149 = t21.*u2.*1.8e3;
t150 = dq1.*dq2.*t120.*9.0e2;
t151 = dq1.*dq3.*t120.*9.0e2;
t152 = dq2.*dq3.*t120.*9.0e2;
t153 = dq1.*dq2.*t32.*2.7e3;
t181 = t134.*2.6487e4;
t182 = t135.*2.943e3;
t183 = t25.*t118.*1.45e3;
t184 = t9.*u3.*9.0e2;
t186 = t21.*u1.*9.0e2;
t187 = dq1.*dq3.*t118.*2.9e3;
t188 = dq2.*dq3.*t118.*2.9e3;
t154 = t53+t55+t57+t59+t63+t64+t65+t66+t67+t69+t70+t71+t72+t74+t75+t76+t77+t78+t79+t80+t81-t116-t117-t119-t121-t122-t123-t124-t125+t133+t137+t139+t140+t141+t142+t143+t144+t145+t146+t147+t148+t149+t150+t151+t152+t153-t181-t182-t183-t184-t186-t187-t188-u2.*7.0e3-t16.*t26.*9.0e2-t16.*t38.*7.5e2-t19.*t38.*7.5e2-t16.*t118.*1.45e3-t19.*t118.*1.45e3-t27.*u2.*5.4e3-t107.*u3.*2.7e3-dq1.*dq2.*t38.*1.5e3-dq1.*dq2.*t118.*2.9e3;
t155 = t114-t132;
t156 = dq1.*t32.*2.7e3;
t157 = dq2.*t32.*2.7e3;
t158 = dq1.*t120.*9.0e2;
t159 = dq2.*t120.*9.0e2;
t160 = dq3.*t120.*9.0e2;
t161 = q1+q2+q3;
t162 = sin(t161);
t163 = t162.*2.4525e3;
t164 = q1+q2-q3;
t165 = sin(t164);
t166 = q1-q2+q3;
t167 = sin(t166);
t168 = t167.*1.32435e4;
t169 = -q1+q2+q3;
t170 = sin(t169);
t171 = t170.*5.886e3;
t172 = t25.*t107.*9.0e2;
t173 = t33.*u2.*1.8e3;
t174 = t4.*t16.*2.7e3;
t175 = t4.*t19.*2.7e3;
t176 = t22.*u2.*2.7e3;
t177 = t26.*u1.*9.0e2;
t178 = dq1.*dq3.*t107.*1.8e3;
t179 = dq2.*dq3.*t107.*1.8e3;
t180 = dq1.*dq2.*t4.*5.4e3;
t189 = t104.*8.829e3;
t190 = t16.*t18.*2.25e3;
t191 = t16.*t107.*9.0e2;
t192 = t19.*t107.*9.0e2;
t193 = t9.*t19.*3.0e2;
t194 = t9.*t25.*3.0e2;
t195 = t34.*u1.*2.1e3;
t196 = t31.*u2.*2.7e3;
t197 = t120.*u3.*5.4e3;
t198 = t25.*t130.*1.45e3;
t199 = dq1.*dq2.*t107.*1.8e3;
t200 = dq1.*dq2.*t9.*6.0e2;
t201 = dq1.*dq3.*t9.*6.0e2;
t202 = dq2.*dq3.*t9.*6.0e2;
t203 = dq1.*dq3.*t130.*2.9e3;
t204 = dq2.*dq3.*t130.*2.9e3;
t205 = u3.*1.9e4;
t206 = cos(t161);
t207 = cos(t164);
t208 = t207.*8.829e3;
t209 = cos(t166);
t210 = cos(t169);
t211 = t210.*5.886e3;
t212 = t16.*t22.*4.95e3;
t213 = t16.*t31.*2.25e3;
t214 = t16.*t120.*9.0e2;
t215 = t19.*t120.*9.0e2;
t216 = t24.*u2.*2.1e3;
t217 = t19.*t33.*1.5e2;
t218 = t25.*t33.*1.5e2;
t219 = t130.*u3.*1.14e4;
t220 = t18.*u1.*2.7e3;
t221 = t107.*u2.*2.7e3;
t222 = dq1.*dq2.*t120.*1.8e3;
t223 = dq1.*dq2.*t33.*3.0e2;
t224 = dq1.*dq3.*t33.*3.0e2;
t225 = dq2.*dq3.*t33.*3.0e2;
t226 = t68+t73+t123+t137+t139+t143+t144+t146+t147+t151+t152+t153-t181-t182-t183-t184-t185-t186-t187-t188+t205-t206.*2.4525e3+t208-t209.*1.32435e4+t211+t212+t213+t214+t215+t216+t217+t218+t219+t220+t221+t222+t223+t224+t225-u2.*4.7e3-t16.*t26.*7.5e2-t16.*t34.*8.5e2-t16.*t38.*1.5e3-t19.*t38.*1.5e3-t25.*t38.*7.5e2-t16.*t118.*5.0e3-t19.*t118.*5.0e3-t4.*u3.*8.1e3-t18.*u2.*2.7e3-t24.*u1.*2.1e3-t107.*u3.*5.4e3-t130.*u2.*5.7e3-dq1.*dq2.*t38.*3.0e3-dq1.*dq3.*t38.*1.5e3-dq2.*dq3.*t38.*1.5e3-dq1.*dq2.*t118.*1.0e4;
t227 = dq1.*t33.*3.0e2;
t228 = dq2.*t33.*3.0e2;
t229 = dq3.*t33.*3.0e2;
t230 = dq1.*t120.*1.8e3;
t231 = dq2.*t120.*1.8e3;
A = reshape([0.0,0.0,0.0,t11.*(t15.*(-1.4715e3)+t91+t92+t93-t94.*3.1392e4).*(3.0./5.0e1),t11.*(t15.*1.4715e3-t91-t92-t93+t94.*3.1392e4+t97+t100-t102.*2.20725e4+t105).*(3.0./5.0e1),t11.*(t97+t100-t102.*2.20725e4+t105+t163-t165.*8.829e3+t168+t171).*(-3.0./5.0e1),0.0,0.0,0.0,t11.*(t13.*-2.6487e4+t37+t39+t40+t41+t42+t43+t44+t45+t46+t47+t48+t49+t50+t51+t110+t111+t112+t174+t176-t9.*t16.*3.0e2-t16.*t21.*1.5e2-t19.*t21.*1.5e2+t16.*t27.*2.25e3-t22.*u3.*2.7e3-t26.*u2.*9.0e2-t31.*u3.*2.7e3-dq1.*dq2.*t21.*3.0e2).*(-3.0./5.0e1)-t52.*t82.*t115.*(3.0./5.0e1),t11.*(t13.*-2.6487e4+t37+t39+t40+t41+t42+t43+t44+t45+t46+t47+t48+t49+t50+t51-t97+t100-t102.*2.20725e4-t105-t108-t109+t110+t111+t112+t129+t172+t175+t177+t178+t179+t180+t191+t192+t197+t199+t4.*t16.*5.4e3-t16.*t21.*9.0e2-t19.*t21.*1.5e2+t16.*t27.*7.2e3-t22.*u1.*2.7e3+t22.*u2.*5.4e3-t22.*u3.*2.7e3-t31.*u3.*2.7e3-t33.*u2.*1.8e3-dq1.*dq2.*t21.*3.0e2).*(3.0./5.0e1)+t52.*t115.*t154.*(3.0./5.0e1),t11.*(-t97+t100-t102.*2.20725e4-t105+t108+t129+t163-t165.*8.829e3-t168-t171+t172-t173+t174+t175+t176+t177+t178+t179+t180+t190+t193+t194+t195+t196+t200+t201+t202-t16.*t21.*7.5e2-t16.*t24.*8.5e2+t16.*t27.*4.95e3+t16.*t107.*1.8e3+t19.*t107.*1.8e3-t22.*u1.*2.7e3-t26.*u2.*9.0e2-t31.*u1.*2.7e3+t32.*u3.*1.62e4-t34.*u2.*2.1e3-t120.*u2.*5.4e3+t120.*u3.*1.08e4+dq1.*dq2.*t107.*3.6e3).*(-3.0./5.0e1)-t52.*t115.*t226.*(3.0./5.0e1),0.0,0.0,0.0,t11.*(t35.*-8.829e3-t37+t39+t40+t41-t42-t43-t44-t45+t46+t47+t48-t49-t50-t51+t108+t109+t126+t127+t128+t131+t16.*t21.*3.0e2-t26.*u3.*1.8e3-t31.*u3.*2.7e3-t38.*u1.*1.8e3).*(3.0./5.0e1)+t52.*t82.*t155.*(3.0./5.0e1),t11.*(t35.*-8.829e3-t37+t39+t40+t41-t42-t43-t44-t45+t46+t47+t48-t49-t50-t51-t99.*5.886e3+t108+t126+t127+t128-t129+t131+t173+t189+t198+t203+t204+t6.*t16.*1.5e3+t6.*t19.*1.5e3+t16.*t21.*1.8e3-t16.*t107.*4.5e2-t19.*t107.*4.5e2-t25.*t107.*4.5e2+t16.*t130.*1.45e3+t19.*t130.*1.45e3-t26.*u1.*1.8e3+t26.*u2.*3.6e3-t26.*u3.*1.8e3-t31.*u3.*2.7e3-t38.*u1.*1.8e3+t118.*u3.*5.7e3-t120.*u3.*2.7e3+dq1.*dq2.*t6.*3.0e3-dq1.*dq2.*t107.*9.0e2-dq1.*dq3.*t107.*9.0e2-dq2.*dq3.*t107.*9.0e2+dq1.*dq2.*t130.*2.9e3).*(-3.0./5.0e1)-t52.*t154.*t155.*(3.0./5.0e1),t11.*(t99.*-5.886e3-t108+t109-t129-t163-t165.*8.829e3-t168+t171+t173+t189+t190-t191-t192-t193-t194-t195+t196-t197+t198-t199-t200-t201-t202+t203+t204+t6.*t16.*3.0e3+t6.*t19.*3.0e3+t6.*t25.*1.5e3+t16.*t21.*1.5e3+t16.*t24.*8.5e2-t25.*t107.*4.5e2+t16.*t130.*5.0e3+t19.*t130.*5.0e3-t26.*u1.*1.8e3-t31.*u1.*2.7e3+t34.*u2.*2.1e3-t118.*u2.*5.7e3+t118.*u3.*1.14e4+t120.*u2.*2.7e3+dq1.*dq2.*t6.*6.0e3+dq1.*dq3.*t6.*3.0e3+dq2.*dq3.*t6.*3.0e3-dq1.*dq3.*t107.*9.0e2-dq2.*dq3.*t107.*9.0e2+dq1.*dq2.*t130.*1.0e4).*(3.0./5.0e1)+t52.*t226.*(t114-t132).*(3.0./5.0e1),1.0,0.0,0.0,t11.*(t83+t84+t85+t86+t87+t88+t89+t90+t156-dq1.*t26.*3.0e2-dq2.*t26.*3.0e2-dq1.*t33.*3.0e2).*(-3.0./5.0e1),t11.*(t83+t84+t85+t87+t88+t89+t90+t157+t158+t159+t160+dq1.*t22.*1.44e4-dq1.*t26.*1.8e3-dq2.*t26.*3.0e2+dq1.*t32.*5.4e3-dq1.*t33.*3.0e2-dq1.*t38.*1.5e3-dq2.*t38.*1.5e3-dq1.*t118.*2.9e3-dq2.*t118.*2.9e3-dq3.*t118.*2.9e3).*(3.0./5.0e1),t11.*(t156+t157+t160+t227+t228+t229+t230+t231+dq1.*t22.*9.9e3-dq1.*t26.*1.5e3+dq1.*t31.*4.5e3-dq1.*t34.*1.7e3-dq1.*t38.*3.0e3-dq2.*t38.*3.0e3-dq3.*t38.*1.5e3-dq1.*t118.*1.0e4-dq2.*t118.*1.0e4-dq3.*t118.*2.9e3).*(-3.0./5.0e1),0.0,1.0,0.0,t11.*(t83+t84+t85+t86+t87+t88+t89+t90-dq1.*t26.*3.0e2-dq2.*t26.*3.0e2).*(-3.0./5.0e1),t11.*(t83+t84+t85+t86+t87+t88+t89+t90+t156+t157+t158+t159+t160-dq1.*t26.*3.0e2-dq2.*t26.*3.0e2-dq1.*t38.*1.5e3-dq2.*t38.*1.5e3-dq1.*t118.*2.9e3-dq2.*t118.*2.9e3-dq3.*t118.*2.9e3).*(3.0./5.0e1),t11.*(t156+t157+t160+t227+t228+t229+t230+t231-dq1.*t38.*3.0e3-dq2.*t38.*3.0e3-dq3.*t38.*1.5e3-dq1.*t118.*1.0e4-dq2.*t118.*1.0e4-dq3.*t118.*2.9e3).*(-3.0./5.0e1),0.0,0.0,1.0,t11.*(t83+t84+t85+t88+t89+t90).*(-3.0./5.0e1),t11.*(t83+t84+t85+t88+t89+t90+t158+t159+t160-dq1.*t118.*2.9e3-dq2.*t118.*2.9e3-dq3.*t118.*2.9e3).*(3.0./5.0e1),t11.*(t158+t159+t160+t227+t228+t229-dq1.*t38.*1.5e3-dq2.*t38.*1.5e3-dq3.*t38.*1.5e3-dq1.*t118.*2.9e3-dq2.*t118.*2.9e3-dq3.*t118.*2.9e3).*(-3.0./5.0e1)],[6,6]);
