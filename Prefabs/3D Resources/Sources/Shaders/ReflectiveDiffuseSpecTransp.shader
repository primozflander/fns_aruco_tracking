Shader "Reflective/Diffuse Reflection Spec Transp" {
Properties {
	_Color ("Main Color", Color) = (1,1,1,1)
	_SpecColor ("Specular Color", Color) = (0.5, 0.5, 0.5, 0)
	_Shininess ("Shininess", Range (0.01, 3)) = 0.078125
	_Gloss("Gloss", Range (0.00, 1)) = .5
	_ReflectColor ("Reflection Color", Color) = (1,1,1,0.5)
	_Reflection("Reflection", Range (0.01, 1)) = .5
	_MainTex ("Base (RGB) RefStrength+Gloss (A)", 2D) = "white" {} 
	_Cube ("Reflection Cubemap", Cube) = "_Skybox" { TexGen CubeReflect }
	_ReflectionAmount ("Fresnel Reflection", Range(0, 10)) = 0.25
	_RimPower ("Fresnel Falloff", Range(0, 10)) = 4
}
SubShader {
	LOD 300
	Tags { "Queue"="Transparent" "IgnoreProjector"="True" "RenderType"="Transperant" }
	
CGPROGRAM
#pragma surface surf BlinnPhong alpha

sampler2D _MainTex;
samplerCUBE _Cube;

fixed4 _Color;
fixed4 _ReflectColor;
half _Shininess;
half _Gloss;
half _Reflection;
float _ReflectionAmount;
float _RimPower;

struct Input {
	float2 uv_MainTex;
	float3 worldRefl;
    float3 viewDir;
};

void surf (Input IN, inout SurfaceOutput o) {
	float rim = 1.0 - saturate(dot(o.Normal, normalize(IN.viewDir)));
	rim = pow(rim, _RimPower);

	fixed4 tex = tex2D(_MainTex, IN.uv_MainTex);
	fixed4 c = tex * _Color;
	fixed4 reflcol = texCUBE (_Cube, IN.worldRefl);
//	reflcol *= tex.a;
	o.Albedo = lerp(c.rgb, reflcol.rgb, _Reflection);
//	o.Gloss = tex.a;
	o.Gloss = _Gloss;
	o.Alpha = _Color.a;
//	o.Alpha = _Color.a * tex.a;
	o.Specular = _Shininess;
	o.Emission = (c.rgb * reflcol.rgb * _ReflectColor.rgb * _ReflectionAmount) * rim;
}
ENDCG
}
	
FallBack "Reflective/VertexLit"
} 
