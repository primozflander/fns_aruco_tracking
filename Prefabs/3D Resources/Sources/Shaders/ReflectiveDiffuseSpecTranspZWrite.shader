Shader "Reflective/Diffuse Reflection Spec Transp ZWrite" {
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
	
    // extra pass that renders to depth buffer only
    Pass {
        ZWrite On
        ColorMask 0
    }

    // paste in forward rendering passes from Transparent/Specular
    UsePass "Reflective/Diffuse Reflection Spec Transp/FORWARD"
}
	
FallBack "Reflective/VertexLit"
} 
