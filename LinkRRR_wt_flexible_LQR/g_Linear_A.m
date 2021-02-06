function A = g_Linear_A(in1,in2)
%G_LINEAR_A
%    A = G_LINEAR_A(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    12-Jun-2020 10:15:45

dq1 = in1(7,:);
dq2 = in1(8,:);
dq3 = in1(9,:);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
qs1 = in1(4,:);
qs2 = in1(5,:);
qs3 = in1(6,:);
t2 = q2.*2.0;
t3 = q3.*2.0;
t4 = cos(t3);
t5 = cos(t2);
t6 = t5.*8.1e1;
t7 = t4.*4.5e1;
t8 = t2+t3;
t9 = cos(t8);
t30 = t9.*9.0;
t10 = t6+t7-t30-1.69e2;
t11 = 1.0./t10;
t12 = q1+t2;
t13 = sin(t12);
t14 = q1+t2+t3;
t15 = sin(t14);
t16 = dq1.^2;
t17 = q2+t3;
t18 = cos(t17);
t19 = q2-q3;
t20 = cos(t19);
t21 = dq2.^2;
t22 = q2+q3;
t23 = sin(t22);
t24 = sin(q2);
t25 = cos(t22);
t26 = dq3.^2;
t27 = sin(t17);
t28 = sin(t19);
t29 = cos(q2);
t31 = q1-t3;
t32 = q1+t3;
t33 = sin(t2);
t34 = sin(t8);
t35 = sin(t31);
t36 = sin(t32);
t37 = t29.*2.7e6;
t38 = t15.*2.943e3;
t39 = sin(t3);
t40 = t16.*t20.*4.5e2;
t41 = t20.*t21.*4.5e2;
t42 = t20.*t26.*4.5e2;
t43 = qs3.*t23.*2.1e6;
t44 = t16.*t25.*2.5e2;
t45 = t21.*t25.*2.5e2;
t46 = t25.*t26.*2.5e2;
t47 = q3.*t28.*2.7e6;
t48 = dq1.*dq2.*t20.*9.0e2;
t49 = dq1.*dq3.*t20.*9.0e2;
t50 = dq2.*dq3.*t20.*9.0e2;
t51 = dq1.*dq2.*t25.*5.0e2;
t52 = dq1.*dq3.*t25.*5.0e2;
t53 = dq2.*dq3.*t25.*5.0e2;
t54 = 1.0./t10.^2;
t55 = q2.*2.3e6;
t56 = qs1.*2.3e6;
t57 = cos(t12);
t58 = t57.*1.32435e4;
t59 = cos(t31);
t60 = t59.*4.4145e3;
t61 = cos(t32);
t62 = t61.*4.4145e3;
t63 = cos(q1);
t64 = cos(t14);
t65 = q1.*t4.*9.0e5;
t66 = qs2.*t4.*9.0e5;
t67 = t16.*t24.*2.25e3;
t68 = t21.*t24.*2.25e3;
t69 = t16.*t28.*4.5e2;
t70 = t21.*t28.*4.5e2;
t71 = t26.*t28.*4.5e2;
t72 = t16.*t33.*1.35e3;
t73 = q3.*t25.*2.1e6;
t74 = q2.*t29.*2.7e6;
t75 = qs3.*t29.*2.7e6;
t76 = q3.*t18.*9.0e5;
t77 = qs2.*t18.*9.0e5;
t78 = qs3.*t20.*2.7e6;
t79 = t16.*t23.*2.5e2;
t80 = t21.*t23.*2.5e2;
t81 = t23.*t26.*2.5e2;
t82 = dq1.*dq2.*t24.*4.5e3;
t83 = dq1.*dq2.*t28.*9.0e2;
t84 = dq1.*dq3.*t28.*9.0e2;
t85 = dq2.*dq3.*t28.*9.0e2;
t86 = dq1.*dq2.*t23.*5.0e2;
t87 = dq1.*dq3.*t23.*5.0e2;
t88 = dq2.*dq3.*t23.*5.0e2;
t129 = q1.*2.3e6;
t130 = t63.*3.1392e4;
t131 = t64.*1.4715e3;
t132 = q2.*t4.*9.0e5;
t133 = qs1.*t4.*9.0e5;
t135 = t21.*t27.*1.5e2;
t136 = qs3.*t25.*2.1e6;
t137 = t16.*t34.*1.5e2;
t138 = q3.*t29.*2.7e6;
t140 = q3.*t20.*2.7e6;
t141 = qs3.*t18.*9.0e5;
t142 = dq1.*dq2.*t27.*3.0e2;
t219 = qs2.*t29.*2.7e6;
t220 = q2.*t18.*9.0e5;
t89 = qs2.*-2.3e6+t55+t56+t58+t60+t62+t65+t66+t67+t68+t69+t70+t71+t72+t73+t74+t75+t76+t77+t78+t79+t80+t81+t82+t83+t84+t85+t86+t87+t88-t129-t130-t131-t132-t133-t135-t136-t137-t138-t140-t141-t142-t219-t220-t16.*t27.*1.5e2;
t90 = t4.*9.0e5;
t91 = t20.*2.7e6;
t92 = t18.*9.0e5;
t93 = dq1.*t23.*5.0e2;
t94 = dq2.*t23.*5.0e2;
t95 = dq3.*t23.*5.0e2;
t96 = dq1.*t24.*4.5e3;
t97 = dq2.*t24.*4.5e3;
t98 = dq1.*t28.*9.0e2;
t99 = dq2.*t28.*9.0e2;
t100 = dq3.*t28.*9.0e2;
t101 = t13.*1.32435e4;
t102 = t35.*4.4145e3;
t103 = t36.*4.4145e3;
t104 = sin(q1);
t105 = q1-q2;
t106 = sin(t105);
t107 = t106.*2.6487e4;
t108 = q1+q2+t3;
t109 = sin(t108);
t110 = t109.*2.943e3;
t111 = q1+q2;
t112 = sin(t111);
t113 = -q1+q2+t3;
t114 = sin(t113);
t115 = t114.*4.4145e3;
t116 = q3+t2;
t117 = cos(t116);
t118 = q3.*t23.*2.1e6;
t119 = t9.*t16.*3.0e2;
t120 = q3.*t24.*2.7e6;
t121 = qs2.*t27.*1.8e6;
t122 = qs3.*t27.*9.0e5;
t123 = sin(t116);
t124 = t21.*t29.*2.25e3;
t125 = dq1.*dq2.*t29.*4.5e3;
t126 = t33.*1.62e2;
t127 = t34.*1.8e1;
t128 = t126-t127;
t134 = sin(q3);
t139 = cos(q3);
t143 = t36.*8.829e3;
t144 = t25.*2.1e6;
t145 = q1.*t39.*1.8e6;
t146 = qs2.*t39.*1.8e6;
t147 = t18.*t21.*3.0e2;
t148 = q2.*t34.*1.8e6;
t149 = qs3.*t34.*1.8e6;
t150 = q3.*t27.*1.8e6;
t151 = dq1.*dq2.*t18.*6.0e2;
t152 = t39.*9.0e1;
t153 = q2.*7.0e6;
t154 = qs3.*4.7e6;
t155 = cos(t105);
t156 = cos(t108);
t157 = cos(t111);
t158 = t157.*2.20725e4;
t159 = cos(t113);
t160 = t159.*4.4145e3;
t161 = t16.*t24.*7.2e3;
t162 = q3.*t9.*9.0e5;
t163 = qs2.*t9.*9.0e5;
t164 = t16.*t123.*4.5e2;
t165 = t21.*t123.*4.5e2;
t166 = t26.*t123.*4.5e2;
t167 = t16.*t33.*2.7e3;
t168 = t21.*t33.*1.35e3;
t169 = q2.*t29.*5.4e6;
t170 = qs1.*t29.*2.7e6;
t171 = qs3.*t139.*5.7e6;
t172 = q1.*t18.*9.0e5;
t173 = q3.*t117.*2.7e6;
t174 = qs2.*t18.*1.8e6;
t175 = dq1.*dq2.*t123.*9.0e2;
t176 = dq1.*dq3.*t123.*9.0e2;
t177 = dq2.*dq3.*t123.*9.0e2;
t178 = dq1.*dq2.*t33.*2.7e3;
t213 = t155.*2.6487e4;
t214 = t156.*2.943e3;
t215 = t26.*t134.*1.45e3;
t216 = q2.*t9.*9.0e5;
t217 = qs3.*t9.*9.0e5;
t218 = q1.*t29.*2.7e6;
t221 = qs1.*t18.*9.0e5;
t222 = dq1.*dq3.*t134.*2.9e3;
t223 = dq2.*dq3.*t134.*2.9e3;
t179 = q3.*-4.7e6-qs2.*7.0e6+t56+t58+t60+t62+t65+t66+t68+t69+t70+t71+t73+t75+t76+t78+t79+t80+t81+t82+t83+t84+t85+t86+t87+t88-t129-t130-t131-t132-t133-t135-t136-t137-t138-t140-t141-t142+t153+t154+t158+t160+t161+t162+t163+t164+t165+t166+t167+t168+t169+t170+t171+t172+t173+t174+t175+t176+t177+t178-t213-t214-t215-t216-t217-t218-t221-t222-t223-q2.*t18.*1.8e6-q3.*t139.*5.7e6-qs2.*t29.*5.4e6-qs3.*t117.*2.7e6-t16.*t27.*9.0e2-t16.*t39.*7.5e2-t21.*t39.*7.5e2-t16.*t134.*1.45e3-t21.*t134.*1.45e3-dq1.*dq2.*t39.*1.5e3-dq1.*dq2.*t134.*2.9e3;
t180 = t127-t152;
t181 = t11.*(t37-t90-t92+2.3e6).*(3.0./5.0e1);
t182 = t29.*5.4e6;
t183 = t9.*9.0e5;
t184 = t139.*5.7e6;
t185 = dq1.*t33.*2.7e3;
t186 = dq2.*t33.*2.7e3;
t187 = dq1.*t123.*9.0e2;
t188 = dq2.*t123.*9.0e2;
t189 = dq3.*t123.*9.0e2;
t190 = q1+q2+q3;
t191 = sin(t190);
t192 = t191.*2.4525e3;
t193 = q1+q2-q3;
t194 = sin(t193);
t195 = q1-q2+q3;
t196 = sin(t195);
t197 = t196.*1.32435e4;
t198 = -q1+q2+q3;
t199 = sin(t198);
t200 = t199.*5.886e3;
t201 = t26.*t117.*9.0e2;
t202 = q3.*t34.*1.8e6;
t203 = qs2.*t34.*1.8e6;
t204 = t5.*t16.*2.7e3;
t205 = t5.*t21.*2.7e3;
t206 = q1.*t24.*2.7e6;
t207 = qs2.*t24.*2.7e6;
t208 = q2.*t27.*9.0e5;
t209 = qs1.*t27.*9.0e5;
t210 = dq1.*dq3.*t117.*1.8e3;
t211 = dq2.*dq3.*t117.*1.8e3;
t212 = dq1.*dq2.*t5.*5.4e3;
t224 = t114.*8.829e3;
t225 = t16.*t20.*2.25e3;
t226 = t16.*t117.*9.0e2;
t227 = t21.*t117.*9.0e2;
t228 = q2.*t23.*2.1e6;
t229 = qs1.*t23.*2.1e6;
t230 = t9.*t21.*3.0e2;
t231 = t9.*t26.*3.0e2;
t232 = q1.*t28.*2.7e6;
t233 = q1.*t27.*1.8e6;
t234 = q2.*t27.*1.8e6;
t235 = qs2.*t28.*2.7e6;
t236 = qs3.*t123.*5.4e6;
t237 = t26.*t139.*1.45e3;
t238 = dq1.*dq2.*t117.*1.8e3;
t239 = dq1.*dq2.*t9.*6.0e2;
t240 = dq1.*dq3.*t9.*6.0e2;
t241 = dq2.*dq3.*t9.*6.0e2;
t242 = dq1.*dq3.*t139.*2.9e3;
t243 = dq2.*dq3.*t139.*2.9e3;
t244 = q2.*4.7e6;
t245 = qs3.*1.9e7;
t246 = cos(t190);
t247 = cos(t193);
t248 = t247.*8.829e3;
t249 = cos(t195);
t250 = cos(t198);
t251 = t250.*5.886e3;
t252 = q3.*t5.*8.1e6;
t253 = t16.*t24.*4.95e3;
t254 = t16.*t28.*2.25e3;
t255 = t16.*t123.*9.0e2;
t256 = t21.*t123.*9.0e2;
t257 = q1.*t25.*2.1e6;
t258 = qs2.*t25.*2.1e6;
t259 = t21.*t34.*1.5e2;
t260 = t26.*t34.*1.5e2;
t261 = q2.*t139.*5.7e6;
t262 = qs3.*t139.*1.14e7;
t263 = q2.*t20.*2.7e6;
t264 = q3.*t117.*5.4e6;
t265 = qs1.*t20.*2.7e6;
t266 = qs2.*t117.*2.7e6;
t267 = dq1.*dq2.*t123.*1.8e3;
t268 = dq1.*dq2.*t34.*3.0e2;
t269 = dq1.*dq3.*t34.*3.0e2;
t270 = dq2.*dq3.*t34.*3.0e2;
t271 = q3.*-1.9e7-qs2.*4.7e6+t72+t74+t77+t137+t158+t160+t162+t163+t166+t168+t170+t172+t176+t177+t178-t213-t214-t215-t216-t217-t218-t219-t220-t221-t222-t223+t244+t245-t246.*2.4525e3+t248-t249.*1.32435e4+t251+t252+t253+t254+t255+t256+t257+t258+t259+t260+t261+t262+t263+t264+t265+t266+t267+t268+t269+t270-q1.*t20.*2.7e6-q2.*t25.*2.1e6-q2.*t117.*2.7e6-q3.*t139.*1.14e7-qs3.*t5.*8.1e6-qs2.*t20.*2.7e6-qs1.*t25.*2.1e6-qs3.*t117.*5.4e6-qs2.*t139.*5.7e6-t16.*t23.*8.5e2-t16.*t27.*7.5e2-t16.*t39.*1.5e3-t21.*t39.*1.5e3-t26.*t39.*7.5e2-t16.*t134.*5.0e3-t21.*t134.*5.0e3-dq1.*dq2.*t39.*3.0e3-dq1.*dq3.*t39.*1.5e3-dq2.*dq3.*t39.*1.5e3-dq1.*dq2.*t134.*1.0e4;
t272 = t37+t91-t92-t144;
t273 = t37+t91-t92-t117.*2.7e6-t144-t183+t184+4.7e6;
t274 = t11.*t273.*(3.0./5.0e1);
t275 = t117.*5.4e6;
t276 = t5.*8.1e6;
t277 = dq1.*t34.*3.0e2;
t278 = dq2.*t34.*3.0e2;
t279 = dq3.*t34.*3.0e2;
t280 = dq1.*t123.*1.8e3;
t281 = dq2.*t123.*1.8e3;
A = reshape([0.0,0.0,0.0,0.0,0.0,0.0,t11.*(t4.*-9.0e5-t15.*1.4715e3+t101+t102+t103-t104.*3.1392e4+2.3e6).*(3.0./5.0e1),t11.*(t15.*1.4715e3-t37+t90+t92-t101-t102-t103+t104.*3.1392e4+t107+t110-t112.*2.20725e4+t115-2.3e6).*(3.0./5.0e1),t11.*(-t37-t91+t92+t107+t110-t112.*2.20725e4+t115+t144+t192-t194.*8.829e3+t197+t200).*(-3.0./5.0e1),1.0e3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t11.*(t4.*-9.0e5-t13.*2.6487e4-t18.*9.0e5+t37+t38+t40+t41+t42+t43+t44+t45+t46+t47+t48+t49+t50+t51+t52+t53+t120+t122+t124+t125+t204+t207+t208-q2.*t24.*2.7e6-q3.*t23.*2.1e6-q3.*t27.*9.0e5-qs3.*t24.*2.7e6-qs2.*t27.*9.0e5-qs3.*t28.*2.7e6-t9.*t16.*3.0e2-t16.*t18.*1.5e2-t18.*t21.*1.5e2+t16.*t29.*2.25e3-dq1.*dq2.*t18.*3.0e2+2.3e6).*(-3.0./5.0e1)-t54.*t89.*t128.*(3.0./5.0e1),t11.*(t9.*-9.0e5-t13.*2.6487e4-t18.*1.8e6+t38+t40+t41+t42+t43+t44+t45+t46+t47+t48+t49+t50+t51+t52+t53-t90-t107+t110-t112.*2.20725e4-t115-t118-t119+t120-t121+t122+t124+t125+t148+t149+t182+t201+t205+t206+t209+t210+t211+t212+t226+t227+t234+t236+t238-q2.*t24.*5.4e6-q1.*t27.*9.0e5-q3.*t27.*9.0e5-q3.*t34.*1.8e6-q3.*t123.*5.4e6-qs1.*t24.*2.7e6+qs2.*t24.*5.4e6-qs3.*t24.*2.7e6-qs3.*t28.*2.7e6-qs2.*t34.*1.8e6+t5.*t16.*5.4e3-t16.*t18.*9.0e2-t18.*t21.*1.5e2+t16.*t29.*7.2e3-dq1.*dq2.*t18.*3.0e2+7.0e6).*(3.0./5.0e1)+t54.*t128.*t179.*(3.0./5.0e1),t11.*(t37+t91-t92-t107+t110-t112.*2.20725e4-t115-t117.*2.7e6+t119-t144+t148+t149-t183+t184+t192-t194.*8.829e3-t197-t200+t201-t202-t203+t204+t205+t206+t207+t208+t209+t210+t211+t212+t225+t228+t229+t230+t231+t232+t235+t239+t240+t241-q1.*t23.*2.1e6-q2.*t24.*2.7e6-q1.*t27.*9.0e5-q2.*t28.*2.7e6-q3.*t33.*1.62e7+q2.*t123.*5.4e6-q3.*t123.*1.08e7-qs1.*t24.*2.7e6-qs2.*t23.*2.1e6-qs1.*t28.*2.7e6-qs2.*t27.*9.0e5+qs3.*t33.*1.62e7-qs2.*t123.*5.4e6+qs3.*t123.*1.08e7-t16.*t18.*7.5e2-t16.*t25.*8.5e2+t16.*t29.*4.95e3+t16.*t117.*1.8e3+t21.*t117.*1.8e3+dq1.*dq2.*t117.*3.6e3+4.7e6).*(-3.0./5.0e1)-t54.*t128.*t271.*(3.0./5.0e1),0.0,1.0e3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t11.*(t18.*-9.0e5-t25.*2.1e6-t35.*8.829e3+t37-t38+t40+t41+t42-t43-t44-t45-t46+t47+t48+t49+t50-t51-t52-t53+t91+t118+t119+t121+t143+t145+t146+t147+t150+t151-q2.*t27.*1.8e6-q2.*t39.*1.8e6-qs3.*t27.*1.8e6-qs3.*t28.*2.7e6-qs1.*t39.*1.8e6+t16.*t18.*3.0e2).*(3.0./5.0e1)+t54.*t89.*t180.*(3.0./5.0e1),t11.*(t9.*-9.0e5-t35.*8.829e3+t37-t38+t40+t41+t42-t43-t44-t45-t46+t47+t48+t49+t50-t51-t52-t53+t91-t92-t109.*5.886e3-t117.*2.7e6+t118+t119+t143-t144+t145+t146+t147-t148-t149+t150+t151+t184+t202+t203+t224+t233+t237+t242+t243-q2.*t27.*3.6e6-q2.*t39.*1.8e6+q3.*t123.*2.7e6-q3.*t134.*5.7e6-qs1.*t27.*1.8e6+qs2.*t27.*3.6e6-qs3.*t27.*1.8e6-qs3.*t28.*2.7e6-qs1.*t39.*1.8e6-qs3.*t123.*2.7e6+qs3.*t134.*5.7e6+t4.*t16.*1.5e3+t4.*t21.*1.5e3+t16.*t18.*1.8e3-t16.*t117.*4.5e2-t21.*t117.*4.5e2-t26.*t117.*4.5e2+t16.*t139.*1.45e3+t21.*t139.*1.45e3+dq1.*dq2.*t4.*3.0e3-dq1.*dq2.*t117.*9.0e2-dq1.*dq3.*t117.*9.0e2-dq2.*dq3.*t117.*9.0e2+dq1.*dq2.*t139.*2.9e3+4.7e6).*(-3.0./5.0e1)-t54.*t179.*t180.*(3.0./5.0e1),t11.*(t109.*5.886e3+t119-t121-t139.*1.14e7+t148+t149+t183+t192+t194.*8.829e3+t197-t200-t202-t203-t224-t225+t226+t227+t228+t229+t230+t231-t232-t233+t234-t235+t236-t237+t238+t239+t240+t241-t242-t243+t275+t276-q1.*t23.*2.1e6+q2.*t28.*2.7e6+q2.*t123.*2.7e6-q3.*t123.*5.4e6-q2.*t134.*5.7e6+q3.*t134.*1.14e7-qs2.*t23.*2.1e6+qs1.*t27.*1.8e6+qs1.*t28.*2.7e6-qs2.*t123.*2.7e6+qs2.*t134.*5.7e6-qs3.*t134.*1.14e7-t4.*t16.*3.0e3-t4.*t21.*3.0e3-t4.*t26.*1.5e3-t16.*t18.*1.5e3-t16.*t25.*8.5e2+t26.*t117.*4.5e2-t16.*t139.*5.0e3-t21.*t139.*5.0e3-dq1.*dq2.*t4.*6.0e3-dq1.*dq3.*t4.*3.0e3-dq2.*dq3.*t4.*3.0e3+dq1.*dq3.*t117.*9.0e2+dq2.*dq3.*t117.*9.0e2-dq1.*dq2.*t139.*1.0e4-1.9e7).*(-3.0./5.0e1)+t54.*t271.*(t127-t152).*(3.0./5.0e1),0.0,0.0,1.0e3,0.0,0.0,0.0,0.0,0.0,0.0,t11.*(t90-2.3e6).*(3.0./5.0e1),t181,t11.*t272.*(-3.0./5.0e1),-1.0e3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t181,t11.*(t18.*1.8e6+t90-t182+t183-7.0e6).*(3.0./5.0e1),t274,0.0,-1.0e3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t11.*t272.*(-3.0./5.0e1),t274,t11.*(t139.*-1.14e7+t183+t275+t276-1.9e7).*(3.0./5.0e1),0.0,0.0,-1.0e3,1.0,0.0,0.0,0.0,0.0,0.0,t11.*(t93+t94+t95+t96+t97+t98+t99+t100+t185-dq1.*t27.*3.0e2-dq2.*t27.*3.0e2-dq1.*t34.*3.0e2).*(-3.0./5.0e1),t11.*(t93+t94+t95+t97+t98+t99+t100+t186+t187+t188+t189+dq1.*t24.*1.44e4-dq1.*t27.*1.8e3-dq2.*t27.*3.0e2+dq1.*t33.*5.4e3-dq1.*t34.*3.0e2-dq1.*t39.*1.5e3-dq2.*t39.*1.5e3-dq1.*t134.*2.9e3-dq2.*t134.*2.9e3-dq3.*t134.*2.9e3).*(3.0./5.0e1),t11.*(t185+t186+t189+t277+t278+t279+t280+t281-dq1.*t23.*1.7e3+dq1.*t24.*9.9e3-dq1.*t27.*1.5e3+dq1.*t28.*4.5e3-dq1.*t39.*3.0e3-dq2.*t39.*3.0e3-dq3.*t39.*1.5e3-dq1.*t134.*1.0e4-dq2.*t134.*1.0e4-dq3.*t134.*2.9e3).*(-3.0./5.0e1),0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,t11.*(t93+t94+t95+t96+t97+t98+t99+t100-dq1.*t27.*3.0e2-dq2.*t27.*3.0e2).*(-3.0./5.0e1),t11.*(t93+t94+t95+t96+t97+t98+t99+t100+t185+t186+t187+t188+t189-dq1.*t27.*3.0e2-dq2.*t27.*3.0e2-dq1.*t39.*1.5e3-dq2.*t39.*1.5e3-dq1.*t134.*2.9e3-dq2.*t134.*2.9e3-dq3.*t134.*2.9e3).*(3.0./5.0e1),t11.*(t185+t186+t189+t277+t278+t279+t280+t281-dq1.*t39.*3.0e3-dq2.*t39.*3.0e3-dq3.*t39.*1.5e3-dq1.*t134.*1.0e4-dq2.*t134.*1.0e4-dq3.*t134.*2.9e3).*(-3.0./5.0e1),0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,t11.*(t93+t94+t95+t98+t99+t100).*(-3.0./5.0e1),t11.*(t93+t94+t95+t98+t99+t100+t187+t188+t189-dq1.*t134.*2.9e3-dq2.*t134.*2.9e3-dq3.*t134.*2.9e3).*(3.0./5.0e1),t11.*(t187+t188+t189+t277+t278+t279-dq1.*t39.*1.5e3-dq2.*t39.*1.5e3-dq3.*t39.*1.5e3-dq1.*t134.*2.9e3-dq2.*t134.*2.9e3-dq3.*t134.*2.9e3).*(-3.0./5.0e1),0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0],[12,12]);