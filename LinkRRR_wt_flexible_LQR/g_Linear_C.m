function C = g_Linear_C(in1,in2)
%G_LINEAR_C
%    C = G_LINEAR_C(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    12-Jun-2020 10:16:23

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
t5 = sin(q2);
t6 = dq1.^2;
t7 = dq2.^2;
t8 = q2-q3;
t9 = sin(t8);
t10 = q2+t3;
t11 = sin(t10);
t12 = q2+q3;
t13 = cos(t12);
t14 = t2+t3;
t15 = cos(q2);
t16 = cos(t10);
t17 = cos(t8);
t18 = sin(t12);
t19 = dq3.^2;
t20 = cos(t2);
t21 = t20.*8.1e1;
t22 = t4.*4.5e1;
t23 = cos(t14);
t31 = t23.*9.0;
t24 = t21+t22-t31-1.69e2;
t25 = 1.0./t24;
t26 = q1-t3;
t27 = q1+t3;
t28 = q1+t2+t3;
t29 = sin(t3);
t30 = sin(t14);
t32 = q2.*2.3e6;
t33 = qs1.*2.3e6;
t34 = q1+t2;
t35 = cos(t34);
t36 = t35.*1.32435e4;
t37 = cos(t26);
t38 = t37.*4.4145e3;
t39 = cos(t27);
t40 = t39.*4.4145e3;
t41 = cos(q1);
t42 = cos(t28);
t43 = q1.*t4.*9.0e5;
t44 = qs2.*t4.*9.0e5;
t45 = t5.*t6.*2.25e3;
t46 = t5.*t7.*2.25e3;
t47 = t6.*t9.*4.5e2;
t48 = t7.*t9.*4.5e2;
t49 = t9.*t19.*4.5e2;
t50 = sin(t2);
t51 = t6.*t50.*1.35e3;
t52 = q3.*t13.*2.1e6;
t53 = q2.*t15.*2.7e6;
t54 = qs3.*t15.*2.7e6;
t55 = q3.*t16.*9.0e5;
t56 = qs2.*t16.*9.0e5;
t57 = qs3.*t17.*2.7e6;
t58 = t6.*t18.*2.5e2;
t59 = t7.*t18.*2.5e2;
t60 = t18.*t19.*2.5e2;
t61 = dq1.*dq2.*t5.*4.5e3;
t62 = dq1.*dq2.*t9.*9.0e2;
t63 = dq1.*dq3.*t9.*9.0e2;
t64 = dq2.*dq3.*t9.*9.0e2;
t65 = dq1.*dq2.*t18.*5.0e2;
t66 = dq1.*dq3.*t18.*5.0e2;
t67 = dq2.*dq3.*t18.*5.0e2;
t81 = q1.*2.3e6;
t82 = qs2.*2.3e6;
t83 = t41.*3.1392e4;
t84 = t42.*1.4715e3;
t85 = q2.*t4.*9.0e5;
t86 = qs1.*t4.*9.0e5;
t87 = t6.*t11.*1.5e2;
t88 = t7.*t11.*1.5e2;
t89 = qs3.*t13.*2.1e6;
t90 = t6.*t30.*1.5e2;
t91 = q3.*t15.*2.7e6;
t92 = qs2.*t15.*2.7e6;
t93 = q2.*t16.*9.0e5;
t94 = q3.*t17.*2.7e6;
t95 = qs3.*t16.*9.0e5;
t96 = dq1.*dq2.*t11.*3.0e2;
t68 = t32+t33+t36+t38+t40+t43+t44+t45+t46+t47+t48+t49+t51+t52+t53+t54+t55+t56+t57+t58+t59+t60+t61+t62+t63+t64+t65+t66+t67-t81-t82-t83-t84-t85-t86-t87-t88-t89-t90-t91-t92-t93-t94-t95-t96;
t69 = t15.*2.7e6;
t70 = sin(t28);
t71 = t6.*t17.*4.5e2;
t72 = t7.*t17.*4.5e2;
t73 = t17.*t19.*4.5e2;
t74 = q3.*t18.*2.1e6;
t75 = t6.*t23.*3.0e2;
t76 = q3.*t9.*2.7e6;
t77 = dq1.*dq2.*t17.*9.0e2;
t78 = dq1.*dq3.*t17.*9.0e2;
t79 = dq2.*dq3.*t17.*9.0e2;
t80 = 1.0./t24.^2;
t97 = sin(t34);
t98 = sin(t26);
t99 = sin(t27);
t100 = t4.*9.0e5;
t101 = dq1.*t18.*5.0e2;
t102 = dq2.*t18.*5.0e2;
t103 = dq3.*t18.*5.0e2;
t104 = dq1.*t9.*9.0e2;
t105 = dq2.*t9.*9.0e2;
t106 = dq3.*t9.*9.0e2;
t107 = t17.*2.7e6;
t108 = t16.*9.0e5;
t109 = dq1.*t5.*4.5e3;
t110 = dq2.*t5.*4.5e3;
t111 = t99.*8.829e3;
t112 = t13.*2.1e6;
t113 = t70.*2.943e3;
t114 = q1.*t29.*1.8e6;
t115 = qs2.*t29.*1.8e6;
t116 = q3+t2;
t117 = cos(t116);
t118 = t7.*t16.*3.0e2;
t119 = qs3.*t18.*2.1e6;
t120 = sin(q3);
t121 = t6.*t13.*2.5e2;
t122 = t7.*t13.*2.5e2;
t123 = t13.*t19.*2.5e2;
t124 = q3.*t11.*1.8e6;
t125 = sin(t116);
t126 = cos(q3);
t127 = dq1.*dq2.*t16.*6.0e2;
t128 = dq1.*dq2.*t13.*5.0e2;
t129 = dq1.*dq3.*t13.*5.0e2;
t130 = dq2.*dq3.*t13.*5.0e2;
t131 = t29.*9.0e1;
t132 = t30.*1.8e1;
t133 = t131-t132;
t134 = q1+q2+t3;
t135 = -q1+q2+t3;
t136 = q1-q2;
t137 = sin(t134);
t138 = q1+q2;
t139 = sin(t135);
t140 = q3.*t30.*1.8e6;
t141 = qs2.*t30.*1.8e6;
t142 = q3.*t5.*2.7e6;
t143 = qs2.*t11.*1.8e6;
t144 = qs3.*t11.*9.0e5;
t145 = t7.*t15.*2.25e3;
t146 = dq1.*dq2.*t15.*4.5e3;
t147 = t50.*1.62e2;
t148 = q2.*7.0e6;
t149 = qs3.*4.7e6;
t150 = cos(t136);
t151 = cos(t134);
t152 = cos(t138);
t153 = t152.*2.20725e4;
t154 = cos(t135);
t155 = t154.*4.4145e3;
t156 = t5.*t6.*7.2e3;
t157 = q3.*t23.*9.0e5;
t158 = qs2.*t23.*9.0e5;
t159 = t6.*t125.*4.5e2;
t160 = t7.*t125.*4.5e2;
t161 = t19.*t125.*4.5e2;
t162 = t6.*t50.*2.7e3;
t163 = t7.*t50.*1.35e3;
t164 = q2.*t15.*5.4e6;
t165 = qs1.*t15.*2.7e6;
t166 = qs3.*t126.*5.7e6;
t167 = q1.*t16.*9.0e5;
t168 = q3.*t117.*2.7e6;
t169 = qs2.*t16.*1.8e6;
t170 = dq1.*dq2.*t125.*9.0e2;
t171 = dq1.*dq3.*t125.*9.0e2;
t172 = dq2.*dq3.*t125.*9.0e2;
t173 = dq1.*dq2.*t50.*2.7e3;
t176 = q3.*4.7e6;
t177 = qs2.*7.0e6;
t178 = t150.*2.6487e4;
t179 = t151.*2.943e3;
t180 = t6.*t120.*1.45e3;
t181 = t7.*t120.*1.45e3;
t182 = t19.*t120.*1.45e3;
t183 = q2.*t23.*9.0e5;
t184 = qs3.*t23.*9.0e5;
t185 = t6.*t11.*9.0e2;
t186 = t6.*t29.*7.5e2;
t187 = t7.*t29.*7.5e2;
t188 = q1.*t15.*2.7e6;
t189 = q3.*t126.*5.7e6;
t190 = qs2.*t15.*5.4e6;
t191 = q2.*t16.*1.8e6;
t192 = qs1.*t16.*9.0e5;
t193 = qs3.*t117.*2.7e6;
t194 = dq1.*dq2.*t120.*2.9e3;
t195 = dq1.*dq3.*t120.*2.9e3;
t196 = dq2.*dq3.*t120.*2.9e3;
t197 = dq1.*dq2.*t29.*1.5e3;
t174 = t33+t36+t38+t40+t43+t44+t46+t47+t48+t49+t52+t54+t55+t57+t58+t59+t60+t61+t62+t63+t64+t65+t66+t67-t81-t83-t84-t85-t86-t88-t89-t90-t91-t94-t95-t96+t148+t149+t153+t155+t156+t157+t158+t159+t160+t161+t162+t163+t164+t165+t166+t167+t168+t169+t170+t171+t172+t173-t176-t177-t178-t179-t180-t181-t182-t183-t184-t185-t186-t187-t188-t189-t190-t191-t192-t193-t194-t195-t196-t197;
t175 = t132-t147;
t198 = t69-t100-t108+2.3e6;
t199 = t126.*5.7e6;
t200 = dq1.*t50.*2.7e3;
t201 = dq1.*t125.*9.0e2;
t202 = dq2.*t125.*9.0e2;
t203 = dq3.*t125.*9.0e2;
t204 = dq2.*t50.*2.7e3;
t205 = sin(t136);
t206 = t97.*1.32435e4;
t207 = t98.*4.4145e3;
t208 = t99.*4.4145e3;
t209 = t137.*2.943e3;
t210 = sin(t138);
t211 = sin(q1);
t212 = t23.*9.0e5;
t213 = t15.*5.4e6;
t214 = t205.*2.6487e4;
t215 = t139.*4.4145e3;
t216 = t19.*t117.*9.0e2;
t217 = q2.*t30.*1.8e6;
t218 = qs3.*t30.*1.8e6;
t219 = t6.*t20.*2.7e3;
t220 = t7.*t20.*2.7e3;
t221 = q1.*t5.*2.7e6;
t222 = qs2.*t5.*2.7e6;
t223 = q2.*t11.*9.0e5;
t224 = qs1.*t11.*9.0e5;
t225 = dq1.*dq3.*t117.*1.8e3;
t226 = dq2.*dq3.*t117.*1.8e3;
t227 = dq1.*dq2.*t20.*5.4e3;
t228 = q1+q2+q3;
t229 = q1+q2-q3;
t230 = q1-q2+q3;
t231 = -q1+q2+q3;
t232 = q2.*4.7e6;
t233 = qs3.*1.9e7;
t234 = cos(t228);
t235 = cos(t229);
t236 = t235.*8.829e3;
t237 = cos(t230);
t238 = cos(t231);
t239 = t238.*5.886e3;
t240 = q3.*t20.*8.1e6;
t241 = t5.*t6.*4.95e3;
t242 = t6.*t9.*2.25e3;
t243 = t6.*t125.*9.0e2;
t244 = t7.*t125.*9.0e2;
t245 = q1.*t13.*2.1e6;
t246 = qs2.*t13.*2.1e6;
t247 = t7.*t30.*1.5e2;
t248 = t19.*t30.*1.5e2;
t249 = q2.*t126.*5.7e6;
t250 = qs3.*t126.*1.14e7;
t251 = q2.*t17.*2.7e6;
t252 = q3.*t117.*5.4e6;
t253 = qs1.*t17.*2.7e6;
t254 = qs2.*t117.*2.7e6;
t255 = dq1.*dq2.*t125.*1.8e3;
t256 = dq1.*dq2.*t30.*3.0e2;
t257 = dq1.*dq3.*t30.*3.0e2;
t258 = dq2.*dq3.*t30.*3.0e2;
t285 = q3.*1.9e7;
t286 = qs2.*4.7e6;
t287 = t234.*2.4525e3;
t288 = t237.*1.32435e4;
t289 = qs3.*t20.*8.1e6;
t290 = t6.*t120.*5.0e3;
t291 = t7.*t120.*5.0e3;
t292 = t6.*t11.*7.5e2;
t293 = t6.*t29.*1.5e3;
t294 = t7.*t29.*1.5e3;
t295 = t19.*t29.*7.5e2;
t296 = q2.*t13.*2.1e6;
t297 = qs1.*t13.*2.1e6;
t298 = q3.*t126.*1.14e7;
t299 = qs2.*t126.*5.7e6;
t300 = q1.*t17.*2.7e6;
t301 = q2.*t117.*2.7e6;
t302 = qs2.*t17.*2.7e6;
t303 = qs3.*t117.*5.4e6;
t304 = t6.*t18.*8.5e2;
t305 = dq1.*dq2.*t120.*1.0e4;
t306 = dq1.*dq2.*t29.*3.0e3;
t307 = dq1.*dq3.*t29.*1.5e3;
t308 = dq2.*dq3.*t29.*1.5e3;
t259 = t51+t53+t56+t90-t92-t93+t153+t155+t157+t158+t161+t163+t165+t167+t171+t172+t173-t178-t179-t182-t183-t184-t188-t192-t195-t196+t232+t233+t236+t239+t240+t241+t242+t243+t244+t245+t246+t247+t248+t249+t250+t251+t252+t253+t254+t255+t256+t257+t258-t285-t286-t287-t288-t289-t290-t291-t292-t293-t294-t295-t296-t297-t298-t299-t300-t301-t302-t303-t304-t305-t306-t307-t308;
t260 = sin(t228);
t261 = t260.*2.4525e3;
t262 = sin(t229);
t263 = sin(t230);
t264 = sin(t231);
t265 = t139.*8.829e3;
t266 = t6.*t17.*2.25e3;
t267 = t6.*t117.*9.0e2;
t268 = t7.*t117.*9.0e2;
t269 = q2.*t18.*2.1e6;
t270 = qs1.*t18.*2.1e6;
t271 = t7.*t23.*3.0e2;
t272 = t19.*t23.*3.0e2;
t273 = q1.*t9.*2.7e6;
t274 = q1.*t11.*1.8e6;
t275 = q2.*t11.*1.8e6;
t276 = qs2.*t9.*2.7e6;
t277 = qs3.*t125.*5.4e6;
t278 = t19.*t126.*1.45e3;
t279 = dq1.*dq2.*t117.*1.8e3;
t280 = dq1.*dq2.*t23.*6.0e2;
t281 = dq1.*dq3.*t23.*6.0e2;
t282 = dq2.*dq3.*t23.*6.0e2;
t283 = dq1.*dq3.*t126.*2.9e3;
t284 = dq2.*dq3.*t126.*2.9e3;
t309 = dq1.*t30.*3.0e2;
t310 = dq2.*t30.*3.0e2;
t311 = dq3.*t30.*3.0e2;
t312 = t69+t107-t108-t112-t117.*2.7e6+t199-t212+4.7e6;
t313 = t262.*8.829e3;
t314 = t263.*1.32435e4;
t315 = t117.*5.4e6;
t316 = t20.*8.1e6;
t317 = t69+t107-t108-t112;
t318 = dq1.*t125.*1.8e3;
t319 = dq2.*t125.*1.8e3;
C = [0.0;0.0;0.0;0.0;0.0;0.0;-q3.*(t25.*(t13.*-2.1e6-t16.*9.0e5+t69-t70.*2.943e3+t71+t72+t73+t74+t75+t76+t77+t78+t79-t98.*8.829e3+t107+t111+t114+t115+t118+t124+t127+t143-q2.*t11.*1.8e6-q2.*t29.*1.8e6-qs3.*t9.*2.7e6-qs3.*t11.*1.8e6-qs3.*t18.*2.1e6-qs1.*t29.*1.8e6-t6.*t13.*2.5e2-t7.*t13.*2.5e2+t6.*t16.*3.0e2-t13.*t19.*2.5e2-dq1.*dq2.*t13.*5.0e2-dq1.*dq3.*t13.*5.0e2-dq2.*dq3.*t13.*5.0e2).*(3.0./5.0e1)-t68.*t80.*t133.*(3.0./5.0e1))-t25.*t68.*(3.0./5.0e1)+q2.*(t25.*(t4.*-9.0e5-t16.*9.0e5+t69+t71+t72+t73-t74-t75+t76+t77+t78+t79-t97.*2.6487e4+t113+t119+t121+t122+t123+t128+t129+t130+t142+t144+t145+t146+t219+t222+t223-q2.*t5.*2.7e6-q3.*t11.*9.0e5-qs3.*t5.*2.7e6-qs3.*t9.*2.7e6-qs2.*t11.*9.0e5+t6.*t15.*2.25e3-t6.*t16.*1.5e2-t7.*t16.*1.5e2-dq1.*dq2.*t16.*3.0e2+2.3e6).*(3.0./5.0e1)-t68.*t80.*t175.*(3.0./5.0e1))-qs1.*t25.*(t100-2.3e6).*(3.0./5.0e1)+dq3.*t25.*(t101+t102+t103+t104+t105+t106).*(3.0./5.0e1)+dq1.*t25.*(t101+t102+t103+t104+t105+t106+t109+t110+t200-dq1.*t11.*3.0e2-dq2.*t11.*3.0e2-dq1.*t30.*3.0e2).*(3.0./5.0e1)-q1.*t25.*(t70.*(-1.4715e3)-t100+t206+t207+t208-t211.*3.1392e4+2.3e6).*(3.0./5.0e1)+dq2.*t25.*(t101+t102+t103+t104+t105+t106+t109+t110-dq1.*t11.*3.0e2-dq2.*t11.*3.0e2).*(3.0./5.0e1)-qs2.*t25.*t198.*(3.0./5.0e1)+qs3.*t25.*t317.*(3.0./5.0e1);t25.*t174.*(3.0./5.0e1)-q2.*(t25.*(t16.*-1.8e6-t23.*9.0e5+t71+t72+t73-t74-t75+t76+t77+t78+t79-t97.*2.6487e4-t100+t113+t119+t121+t122+t123+t128+t129+t130-t139.*4.4145e3-t140-t141+t142-t143+t144+t145+t146-t205.*2.6487e4+t209-t210.*2.20725e4+t213+t216+t217+t218+t220+t221+t224+t225+t226+t227+t267+t268+t275+t277+t279-q2.*t5.*5.4e6-q1.*t11.*9.0e5-q3.*t11.*9.0e5-q3.*t125.*5.4e6-qs1.*t5.*2.7e6+qs2.*t5.*5.4e6-qs3.*t5.*2.7e6-qs3.*t9.*2.7e6+t6.*t15.*7.2e3-t6.*t16.*9.0e2-t7.*t16.*1.5e2+t6.*t20.*5.4e3-dq1.*dq2.*t16.*3.0e2+7.0e6).*(3.0./5.0e1)-t80.*t174.*t175.*(3.0./5.0e1))+q3.*(t25.*(t23.*-9.0e5+t69+t71+t72+t73+t74+t75+t76+t77+t78+t79-t98.*8.829e3+t107-t108+t111-t112-t113+t114+t115-t117.*2.7e6+t118-t119-t121-t122-t123+t124+t127-t128-t129-t130-t137.*5.886e3+t140+t141+t199+t265+t274+t278+t283+t284-q2.*t11.*3.6e6-q2.*t29.*1.8e6-q2.*t30.*1.8e6-q3.*t120.*5.7e6+q3.*t125.*2.7e6-qs1.*t11.*1.8e6-qs3.*t9.*2.7e6+qs2.*t11.*3.6e6-qs3.*t11.*1.8e6-qs1.*t29.*1.8e6-qs3.*t30.*1.8e6+qs3.*t120.*5.7e6-qs3.*t125.*2.7e6+t4.*t6.*1.5e3+t4.*t7.*1.5e3+t6.*t16.*1.8e3-t6.*t117.*4.5e2-t7.*t117.*4.5e2+t6.*t126.*1.45e3+t7.*t126.*1.45e3-t19.*t117.*4.5e2+dq1.*dq2.*t4.*3.0e3-dq1.*dq2.*t117.*9.0e2-dq1.*dq3.*t117.*9.0e2-dq2.*dq3.*t117.*9.0e2+dq1.*dq2.*t126.*2.9e3+4.7e6).*(3.0./5.0e1)-t80.*t133.*t174.*(3.0./5.0e1))-dq2.*t25.*(t101+t102+t103+t104+t105+t106+t109+t110+t200+t201+t202+t203+t204-dq1.*t11.*3.0e2-dq2.*t11.*3.0e2-dq1.*t29.*1.5e3-dq2.*t29.*1.5e3-dq1.*t120.*2.9e3-dq2.*t120.*2.9e3-dq3.*t120.*2.9e3).*(3.0./5.0e1)-qs2.*t25.*(t16.*1.8e6+t100+t212-t213-7.0e6).*(3.0./5.0e1)-dq3.*t25.*(t101+t102+t103+t104+t105+t106+t201+t202+t203-dq1.*t120.*2.9e3-dq2.*t120.*2.9e3-dq3.*t120.*2.9e3).*(3.0./5.0e1)-q1.*t25.*(-t69+t70.*1.4715e3+t100+t108-t206-t207-t208+t209-t210.*2.20725e4+t211.*3.1392e4+t214+t215-2.3e6).*(3.0./5.0e1)-qs1.*t25.*t198.*(3.0./5.0e1)-qs3.*t25.*t312.*(3.0./5.0e1)-dq1.*t25.*(t101+t102+t103+t104+t105+t106+t110+t201+t202+t203+t204+dq1.*t5.*1.44e4-dq1.*t11.*1.8e3-dq2.*t11.*3.0e2-dq1.*t29.*1.5e3-dq1.*t30.*3.0e2-dq2.*t29.*1.5e3+dq1.*t50.*5.4e3-dq1.*t120.*2.9e3-dq2.*t120.*2.9e3-dq3.*t120.*2.9e3).*(3.0./5.0e1);t25.*t259.*(-3.0./5.0e1)+q3.*(t25.*(t75-t126.*1.14e7+t137.*5.886e3-t140-t141-t143+t212+t217+t218+t261-t264.*5.886e3-t265-t266+t267+t268+t269+t270+t271+t272-t273-t274+t275-t276+t277-t278+t279+t280+t281+t282-t283-t284+t313+t314+t315+t316+q2.*t9.*2.7e6-q1.*t18.*2.1e6-q2.*t120.*5.7e6+q3.*t120.*1.14e7+q2.*t125.*2.7e6-q3.*t125.*5.4e6+qs1.*t9.*2.7e6+qs1.*t11.*1.8e6-qs2.*t18.*2.1e6+qs2.*t120.*5.7e6-qs3.*t120.*1.14e7-qs2.*t125.*2.7e6-t4.*t6.*3.0e3-t4.*t7.*3.0e3-t6.*t13.*8.5e2-t6.*t16.*1.5e3-t4.*t19.*1.5e3-t6.*t126.*5.0e3-t7.*t126.*5.0e3+t19.*t117.*4.5e2-dq1.*dq2.*t4.*6.0e3-dq1.*dq3.*t4.*3.0e3-dq2.*dq3.*t4.*3.0e3+dq1.*dq3.*t117.*9.0e2+dq2.*dq3.*t117.*9.0e2-dq1.*dq2.*t126.*1.0e4-1.9e7).*(3.0./5.0e1)+t80.*t133.*t259.*(3.0./5.0e1))+q2.*(t25.*(t69+t75+t107-t108-t112-t117.*2.7e6-t140-t141+t199+t209-t210.*2.20725e4-t212-t214-t215+t216+t217+t218+t219+t220+t221+t222+t223+t224+t225+t226+t227+t261-t262.*8.829e3-t263.*1.32435e4-t264.*5.886e3+t266+t269+t270+t271+t272+t273+t276+t280+t281+t282-q2.*t5.*2.7e6-q2.*t9.*2.7e6-q1.*t11.*9.0e5-q1.*t18.*2.1e6-q3.*t50.*1.62e7+q2.*t125.*5.4e6-q3.*t125.*1.08e7-qs1.*t5.*2.7e6-qs1.*t9.*2.7e6-qs2.*t11.*9.0e5-qs2.*t18.*2.1e6+qs3.*t50.*1.62e7-qs2.*t125.*5.4e6+qs3.*t125.*1.08e7-t6.*t13.*8.5e2+t6.*t15.*4.95e3-t6.*t16.*7.5e2+t6.*t117.*1.8e3+t7.*t117.*1.8e3+dq1.*dq2.*t117.*3.6e3+4.7e6).*(3.0./5.0e1)-t80.*t175.*t259.*(3.0./5.0e1))+dq1.*t25.*(t200+t203+t204+t309+t310+t311+t318+t319+dq1.*t5.*9.9e3+dq1.*t9.*4.5e3-dq1.*t11.*1.5e3-dq1.*t18.*1.7e3-dq1.*t29.*3.0e3-dq2.*t29.*3.0e3-dq3.*t29.*1.5e3-dq1.*t120.*1.0e4-dq2.*t120.*1.0e4-dq3.*t120.*2.9e3).*(3.0./5.0e1)+dq2.*t25.*(t200+t203+t204+t309+t310+t311+t318+t319-dq1.*t29.*3.0e3-dq2.*t29.*3.0e3-dq3.*t29.*1.5e3-dq1.*t120.*1.0e4-dq2.*t120.*1.0e4-dq3.*t120.*2.9e3).*(3.0./5.0e1)+dq3.*t25.*(t201+t202+t203+t309+t310+t311-dq1.*t29.*1.5e3-dq2.*t29.*1.5e3-dq3.*t29.*1.5e3-dq1.*t120.*2.9e3-dq2.*t120.*2.9e3-dq3.*t120.*2.9e3).*(3.0./5.0e1)-qs2.*t25.*t312.*(3.0./5.0e1)+qs1.*t25.*t317.*(3.0./5.0e1)-qs3.*t25.*(t126.*-1.14e7+t212+t315+t316-1.9e7).*(3.0./5.0e1)+q1.*t25.*(-t69-t107+t108+t112+t209-t210.*2.20725e4+t214+t215+t261+t264.*5.886e3-t313+t314).*(3.0./5.0e1);0.0;0.0;0.0];
